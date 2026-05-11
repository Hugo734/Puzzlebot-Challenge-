"""CLI training script for the VQ codebooks.

Usage (after building and sourcing the workspace)::

    ros2 run voice_control train

Or directly::

    python3 -m voice_control.train [--training-dir PATH] [--codebook-dir PATH]
                                   [--n-clusters N] [--lpc-order N]

Training data layout::

    <training_dir>/
        start/
            start_00.wav
            start_01.wav
            ...
        stop/
            stop_00.wav
            ...
        <word>/
            ...
"""

from __future__ import annotations

import argparse
import logging
import os
from pathlib import Path

import numpy as np

from voice_control.command_parser import VOCABULARY
from voice_control.lpc import extract_lsf_features
from voice_control.vq import train_all_codebooks

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)


def _load_wav(path: str) -> np.ndarray:
    """Load a WAV file and return a float32 mono signal."""
    try:
        import scipy.io.wavfile as wavfile  # noqa: PLC0415

        rate, data = wavfile.read(path)
        if data.ndim > 1:
            data = data[:, 0]  # take left channel
        if data.dtype == np.int16:
            return data.astype(np.float32) / 32768.0
        if data.dtype == np.int32:
            return data.astype(np.float32) / 2147483648.0
        return data.astype(np.float32)
    except Exception as exc:
        logger.warning('Could not load %s: %s', path, exc)
        return np.array([], dtype=np.float32)


def _collect_features(
    training_dir: str,
    vocabulary: list[str],
    lpc_order: int,
    sample_rate: int,
    frame_ms: int,
    hop_ms: int,
) -> dict[str, np.ndarray]:
    """Walk training_dir and extract LSF features for every known word.

    Returns:
        Mapping ``{word: feature_matrix}`` where each matrix has shape
        ``(n_frames, lpc_order)``.
    """
    training_data: dict[str, np.ndarray] = {}

    for word in vocabulary:
        word_dir = os.path.join(training_dir, word)
        if not os.path.isdir(word_dir):
            logger.warning('No training directory found for word "%s" at %s', word, word_dir)
            continue

        wav_files = sorted(Path(word_dir).glob('*.wav'))
        if not wav_files:
            logger.warning('No WAV files found for word "%s" in %s', word, word_dir)
            continue

        all_features: list[np.ndarray] = []
        for wav_path in wav_files:
            signal = _load_wav(str(wav_path))
            if len(signal) == 0:
                continue
            feats = extract_lsf_features(
                signal,
                sample_rate=sample_rate,
                order=lpc_order,
                frame_ms=frame_ms,
                hop_ms=hop_ms,
            )
            if feats.shape[0] > 0:
                all_features.append(feats)

        if not all_features:
            logger.warning('No valid features extracted for word "%s"', word)
            continue

        combined = np.vstack(all_features)
        training_data[word] = combined
        logger.info(
            'Word "%s": %d WAV files → %d feature frames',
            word,
            len(wav_files),
            combined.shape[0],
        )

    return training_data


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Train VQ codebooks for voice_control LPC+VQ recognizer.'
    )
    parser.add_argument(
        '--training-dir',
        default=os.path.join(os.path.dirname(__file__), '..', 'codebooks', 'training'),
        help='Root directory containing one sub-folder per vocabulary word.',
    )
    parser.add_argument(
        '--codebook-dir',
        default=os.path.join(os.path.dirname(__file__), '..', 'codebooks'),
        help='Output directory for trained codebook pickle files.',
    )
    parser.add_argument('--n-clusters', type=int, default=32, help='VQ codewords per word.')
    parser.add_argument('--lpc-order', type=int, default=12, help='LPC/LSF order.')
    parser.add_argument('--sample-rate', type=int, default=16000, help='Audio sample rate.')
    parser.add_argument('--frame-ms', type=int, default=25, help='Frame length in ms.')
    parser.add_argument('--hop-ms', type=int, default=10, help='Hop size in ms.')
    args = parser.parse_args()

    training_dir = os.path.realpath(args.training_dir)
    codebook_dir = os.path.realpath(args.codebook_dir)

    logger.info('Training directory : %s', training_dir)
    logger.info('Codebook directory : %s', codebook_dir)
    logger.info('Vocabulary         : %s', ', '.join(VOCABULARY))
    logger.info('LPC order          : %d', args.lpc_order)
    logger.info('VQ clusters        : %d', args.n_clusters)

    training_data = _collect_features(
        training_dir=training_dir,
        vocabulary=VOCABULARY,
        lpc_order=args.lpc_order,
        sample_rate=args.sample_rate,
        frame_ms=args.frame_ms,
        hop_ms=args.hop_ms,
    )

    if not training_data:
        logger.error('No training data found — cannot train codebooks.')
        raise SystemExit(1)

    train_all_codebooks(
        training_data=training_data,
        n_clusters=args.n_clusters,
        codebook_dir=codebook_dir,
    )

    logger.info('Training complete.  Codebooks saved to: %s', codebook_dir)
    logger.info('Summary:')
    for word, feats in sorted(training_data.items()):
        logger.info('  %-15s %5d frames', word, feats.shape[0])


if __name__ == '__main__':
    main()
