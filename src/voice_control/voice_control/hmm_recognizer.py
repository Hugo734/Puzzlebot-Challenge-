"""
HMM-based voice recognizer.

Loads the global VQ codebook (K=256) and one HMMBakis model per word,
then classifies a raw audio signal by picking the word with the
highest Forward log-likelihood.
"""

from __future__ import annotations

import os
import numpy as np

from voice_control.hmm_utils import (
    PALABRAS, FS, extract_mfcc, quantize, HMMBakis,
)

_DEFAULT_MODELS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'models',
)


class HMMRecognizer:
    """Load models once, recognise audio on demand."""

    def __init__(self, models_dir: str | None = None):
        self._models_dir = models_dir or _DEFAULT_MODELS_DIR
        self._centroids: np.ndarray | None = None  # (K, D)
        self._hmms: dict[str, HMMBakis] = {}
        self._ready = False

    def load(self) -> bool:
        """Load VQ codebook and all HMM models. Returns True on success."""
        vq_path = os.path.join(self._models_dir, 'global_vq_K256.npz')
        if not os.path.exists(vq_path):
            return False

        data = np.load(vq_path)
        self._centroids = data['centroids']

        hmm_dir = os.path.join(self._models_dir, 'hmm_models')
        loaded = []
        for word in PALABRAS:
            path = os.path.join(hmm_dir, f'{word}_hmm.npz')
            if os.path.exists(path):
                self._hmms[word] = HMMBakis.load(path)
                loaded.append(word)

        self._ready = len(self._hmms) > 0
        return self._ready

    @property
    def is_ready(self) -> bool:
        return self._ready

    @property
    def vocabulary(self) -> list[str]:
        return list(self._hmms.keys())

    def recognize(self, signal: np.ndarray, fs: int = FS) -> str | None:
        """Recognise a single utterance.

        Args:
            signal: 1-D float32 audio samples
            fs:     sample rate (default 16 000 Hz)

        Returns:
            Recognised word string, or None if models not loaded / empty signal.
        """
        if not self._ready or self._centroids is None:
            return None
        if len(signal) < 160:
            return None

        mfcc = extract_mfcc(signal.astype(np.float32), fs=fs)
        if len(mfcc) == 0:
            return None

        indices = quantize(mfcc, self._centroids)
        best_word  = None
        best_score = -np.inf
        for word, hmm in self._hmms.items():
            score = hmm.log_likelihood(indices)
            if score > best_score:
                best_score = score
                best_word  = word

        return best_word
