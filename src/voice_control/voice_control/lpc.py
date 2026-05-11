"""LPC analysis and LSF conversion for voice feature extraction.

Pipeline:
    signal → pre-emphasis → framing → Hamming → autocorrelation
           → Levinson-Durbin → LPC coefficients → LSF features
"""

from __future__ import annotations

import numpy as np

from voice_control.preprocess import frame_signal, hamming_frame, pre_emphasis


def autocorrelation(frame: np.ndarray, order: int) -> np.ndarray:
    """Compute autocorrelation coefficients r[0..order] of a frame.

    Args:
        frame: 1-D windowed audio frame.
        order: LPC order.

    Returns:
        1-D array of length ``order + 1`` containing r[0] through r[order].
    """
    r = np.zeros(order + 1, dtype=np.float64)
    n = len(frame)
    for k in range(order + 1):
        r[k] = float(np.dot(frame[: n - k], frame[k:]))
    return r


def levinson_durbin(r: np.ndarray, order: int) -> np.ndarray:
    """Levinson-Durbin recursion to compute LPC coefficients.

    Args:
        r:     Autocorrelation array of length >= ``order + 1``.
        order: LPC order.

    Returns:
        1-D array of length ``order`` containing LPC coefficients a[1..order].
        Returns zeros if r[0] is too small (silence / near-silence frame).
    """
    if abs(r[0]) < 1e-10:
        return np.zeros(order, dtype=np.float64)

    a = np.zeros(order, dtype=np.float64)
    e = float(r[0])

    for i in range(order):
        # Compute reflection coefficient
        acc = r[i + 1]
        for j in range(i):
            acc -= a[j] * r[i - j]
        k = acc / e

        # Update coefficients
        a_new = a.copy()
        a_new[i] = k
        for j in range(i):
            a_new[j] = a[j] - k * a[i - 1 - j]
        a = a_new

        # Update prediction error
        e = e * (1.0 - k * k)
        if e <= 0.0:
            break

    return a


def lpc_to_lsf(lpc: np.ndarray) -> np.ndarray:
    """Convert LPC coefficients to Line Spectral Frequencies (LSF).

    LSFs are derived from the sum and difference polynomials P and Q of the
    LPC polynomial A(z).  The roots of P and Q alternate on the unit circle,
    giving a stable and robust feature representation.

    Args:
        lpc: 1-D array of LPC coefficients a[1..order] (length == order).

    Returns:
        1-D array of length ``order`` containing LSF values in [0, π],
        sorted in ascending order.  If root computation fails, returns an
        evenly spaced fallback.
    """
    order = len(lpc)
    # Build A(z) = 1 + a[0]*z^-1 + ... + a[order-1]*z^-order
    a_poly = np.concatenate([[1.0], lpc])

    # P(z) = A(z) + z^-(order+1) * A(z^-1)
    # Q(z) = A(z) - z^-(order+1) * A(z^-1)
    a_rev = a_poly[::-1]
    p_poly = a_poly + a_rev
    q_poly = a_poly - a_rev

    # Remove the trivial roots at z = 1 (P) and z = -1 (Q)
    # P has factor (1 - z^-1), Q has factor (1 + z^-1)
    p_reduced = np.polydiv(p_poly, [1.0, -1.0])[0]
    q_reduced = np.polydiv(q_poly, [1.0, 1.0])[0]

    try:
        p_roots = np.roots(p_reduced)
        q_roots = np.roots(q_reduced)

        # Keep roots on or near the unit circle with positive imaginary part
        p_angles = np.angle(p_roots[np.imag(p_roots) >= 0.0])
        q_angles = np.angle(q_roots[np.imag(q_roots) >= 0.0])

        lsf = np.sort(np.concatenate([p_angles, q_angles]))
        lsf = lsf[lsf >= 0.0]

        # Pad or truncate to exactly `order` values
        if len(lsf) < order:
            lsf = np.pad(lsf, (0, order - len(lsf)), constant_values=np.pi)
        else:
            lsf = lsf[:order]

        return lsf.astype(np.float32)

    except (np.linalg.LinAlgError, ValueError):
        # Fallback: evenly spaced LSFs
        return np.linspace(0.0, np.pi, order, dtype=np.float32)


def extract_lsf_features(
    signal: np.ndarray,
    sample_rate: int = 16000,
    order: int = 12,
    frame_ms: int = 25,
    hop_ms: int = 10,
) -> np.ndarray:
    """Full pipeline: raw signal → LSF feature matrix.

    Steps:
        1. Pre-emphasis filtering
        2. Framing with overlap
        3. Hamming windowing per frame
        4. Autocorrelation + Levinson-Durbin → LPC
        5. LPC → LSF conversion

    Args:
        signal:     1-D float32 audio signal.
        sample_rate: Sampling rate in Hz.
        order:      LPC / LSF order.
        frame_ms:   Frame length in milliseconds.
        hop_ms:     Hop size in milliseconds.

    Returns:
        2-D float32 array of shape ``(n_frames, order)``.
        Returns empty array of shape ``(0, order)`` for very short signals.
    """
    frame_len = int(sample_rate * frame_ms / 1000)
    hop_len = int(sample_rate * hop_ms / 1000)

    emphasised = pre_emphasis(signal.astype(np.float64))
    frames = frame_signal(emphasised, frame_len, hop_len)

    if frames.shape[0] == 0:
        return np.empty((0, order), dtype=np.float32)

    lsf_matrix = np.zeros((frames.shape[0], order), dtype=np.float32)
    for i, frame in enumerate(frames):
        windowed = hamming_frame(frame)
        r = autocorrelation(windowed, order)
        lpc = levinson_durbin(r, order)
        lsf_matrix[i] = lpc_to_lsf(lpc)

    return lsf_matrix
