"""Signal pre-processing utilities for voice recognition.

Functions here operate on raw float32 audio samples and prepare them for
LPC feature extraction.
"""

from __future__ import annotations

import numpy as np


def pre_emphasis(signal: np.ndarray, alpha: float = 0.97) -> np.ndarray:
    """Apply a first-order high-pass pre-emphasis filter.

    y[n] = x[n] - alpha * x[n-1]

    Args:
        signal: 1-D float32 input signal.
        alpha:  Pre-emphasis coefficient (default 0.97).

    Returns:
        Pre-emphasised signal of the same shape.
    """
    emphasised = np.empty_like(signal)
    emphasised[0] = signal[0]
    emphasised[1:] = signal[1:] - alpha * signal[:-1]
    return emphasised


def hamming_frame(signal: np.ndarray) -> np.ndarray:
    """Multiply a signal frame by a Hamming window.

    Args:
        signal: 1-D array representing a single analysis frame.

    Returns:
        Windowed frame of the same shape.
    """
    return signal * np.hamming(len(signal))


def frame_signal(
    signal: np.ndarray,
    frame_len: int,
    hop_len: int,
) -> np.ndarray:
    """Segment a 1-D signal into overlapping frames.

    Args:
        signal:    1-D float32 input signal.
        frame_len: Length of each frame in samples.
        hop_len:   Hop size (step) between successive frames in samples.

    Returns:
        2-D array of shape ``(n_frames, frame_len)``.  If the signal is shorter
        than one frame, returns an empty array of shape ``(0, frame_len)``.
    """
    if len(signal) < frame_len:
        return np.empty((0, frame_len), dtype=signal.dtype)

    n_frames = 1 + (len(signal) - frame_len) // hop_len
    # Use stride tricks for zero-copy slicing
    shape = (n_frames, frame_len)
    strides = (signal.strides[0] * hop_len, signal.strides[0])
    return np.lib.stride_tricks.as_strided(signal, shape=shape, strides=strides).copy()
