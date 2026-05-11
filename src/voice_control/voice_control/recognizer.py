"""VQ-based voice recognizer.

Loads one VQCodebook per vocabulary word and classifies a raw audio utterance
by finding the codebook with the minimum average distortion.
"""

from __future__ import annotations

import logging
import os

import numpy as np

from voice_control.lpc import extract_lsf_features
from voice_control.vq import VQCodebook

logger = logging.getLogger(__name__)


class VoiceRecognizer:
    """Classifies utterances against per-word VQ codebooks.

    Args:
        codebook_dir:        Directory containing ``{word}.pkl`` codebook files.
        vocabulary:          Ordered list of words to recognise.
        lpc_order:           LPC order used during feature extraction (must match
                             the order used at training time).
        frame_ms:            Frame length in milliseconds for LSF extraction.
        hop_ms:              Hop size in milliseconds for LSF extraction.
        sample_rate:         Expected sampling rate of utterances.
        rejection_threshold: Minimum distortion above which recognition returns
                             ``None`` (utterance rejected as out-of-vocabulary).
    """

    def __init__(
        self,
        codebook_dir: str,
        vocabulary: list[str],
        lpc_order: int = 12,
        frame_ms: int = 25,
        hop_ms: int = 10,
        sample_rate: int = 16000,
        rejection_threshold: float = 2.0,
    ) -> None:
        self._codebook_dir = codebook_dir
        self._vocabulary = vocabulary
        self._lpc_order = lpc_order
        self._frame_ms = frame_ms
        self._hop_ms = hop_ms
        self._sample_rate = sample_rate
        self._rejection_threshold = rejection_threshold

        self._codebooks: dict[str, VQCodebook] = {}

    # ── Lifecycle ─────────────────────────────────────────────────────

    def load_codebooks(self) -> None:
        """Load all codebook PKL files from ``codebook_dir``.

        Words whose PKL file is missing are skipped with a warning.
        """
        self._codebooks = {}
        for word in self._vocabulary:
            path = os.path.join(self._codebook_dir, f'{word}.pkl')
            if not os.path.isfile(path):
                logger.warning('Codebook not found for word "%s" at %s', word, path)
                continue
            cb = VQCodebook()
            cb.load(path)
            self._codebooks[word] = cb
            logger.debug('Loaded codebook for "%s"', word)

        logger.info(
            'VoiceRecognizer ready — %d/%d codebooks loaded',
            len(self._codebooks),
            len(self._vocabulary),
        )

    @property
    def is_ready(self) -> bool:
        """True when all vocabulary codebooks are loaded."""
        return len(self._codebooks) == len(self._vocabulary)

    # ── Recognition ───────────────────────────────────────────────────

    def recognize(self, utterance: np.ndarray) -> str | None:
        """Classify a raw audio utterance.

        Args:
            utterance: 1-D float32 audio signal (normalised to [-1, 1]).

        Returns:
            The recognised word string, or ``None`` if:
            - no codebooks are loaded, or
            - the minimum distortion exceeds ``rejection_threshold``.
        """
        if not self._codebooks:
            logger.error('No codebooks loaded — call load_codebooks() first')
            return None

        features = extract_lsf_features(
            utterance,
            sample_rate=self._sample_rate,
            order=self._lpc_order,
            frame_ms=self._frame_ms,
            hop_ms=self._hop_ms,
        )

        if features.shape[0] == 0:
            logger.warning('Utterance too short to extract features')
            return None

        distortions: dict[str, float] = {
            word: cb.distortion(features)
            for word, cb in self._codebooks.items()
        }

        best_word = min(distortions, key=lambda w: distortions[w])
        best_dist = distortions[best_word]

        logger.debug(
            'Recognition distortions: %s  →  best="%s" (%.4f)',
            {w: f'{d:.4f}' for w, d in distortions.items()},
            best_word,
            best_dist,
        )

        if best_dist > self._rejection_threshold:
            logger.info(
                'Utterance rejected (distortion %.4f > threshold %.4f)',
                best_dist,
                self._rejection_threshold,
            )
            return None

        return best_word
