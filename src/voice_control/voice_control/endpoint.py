"""Voice activity detection (VAD) / endpointing module.

Uses an adaptive energy threshold to detect speech onset and offset.
"""

from __future__ import annotations

import numpy as np


class VoiceEndpointer:
    """Energy-based voice activity detector with adaptive threshold.

    Args:
        sample_rate:              Samples per second (default 16 000).
        frame_ms:                 Analysis frame length in milliseconds (default 25).
        energy_threshold_factor:  Multiplier over running mean energy to decide
                                  speech onset (default 3.0).
        min_speech_ms:            Minimum speech duration to accept an utterance
                                  (default 200 ms).
        silence_ms:               Silence duration after speech that triggers
                                  end-of-utterance (default 500 ms).
    """

    def __init__(
        self,
        sample_rate: int = 16000,
        frame_ms: int = 25,
        energy_threshold_factor: float = 3.0,
        min_speech_ms: int = 200,
        silence_ms: int = 500,
    ) -> None:
        self._sample_rate = sample_rate
        self._frame_len = int(sample_rate * frame_ms / 1000)
        self._energy_factor = energy_threshold_factor
        self._min_speech_frames = max(1, int(min_speech_ms / frame_ms))
        self._silence_frames = max(1, int(silence_ms / frame_ms))

        # State
        self._buffer: list[np.ndarray] = []
        self._speech_frames: list[np.ndarray] = []
        self._in_speech: bool = False
        self._silence_count: int = 0
        self._speech_count: int = 0

        # Adaptive energy baseline
        self._energy_history: list[float] = []
        self._history_len: int = 50  # frames

    # ── Public API ────────────────────────────────────────────────────

    def process(self, chunk: np.ndarray) -> np.ndarray | None:
        """Feed one audio chunk and return a complete utterance when detected.

        Internally segments the chunk into frames of size ``frame_len``.
        Leftover samples are buffered for the next call.

        Args:
            chunk: 1-D float32 audio samples.

        Returns:
            Concatenated speech frames as a float32 array, or ``None`` if no
            complete utterance has been detected yet.
        """
        self._buffer.append(chunk)
        combined = np.concatenate(self._buffer)

        n_frames = len(combined) // self._frame_len
        if n_frames == 0:
            return None

        frames = combined[: n_frames * self._frame_len].reshape(n_frames, self._frame_len)
        remainder = combined[n_frames * self._frame_len :]
        self._buffer = [remainder] if len(remainder) > 0 else []

        utterance: np.ndarray | None = None

        for frame in frames:
            energy = float(np.mean(frame ** 2))
            self._update_energy_history(energy)
            threshold = self._compute_threshold()

            if not self._in_speech:
                if energy > threshold:
                    self._in_speech = True
                    self._silence_count = 0
                    self._speech_count = 1
                    self._speech_frames = [frame]
            else:
                if energy > threshold:
                    self._speech_frames.append(frame)
                    self._speech_count += 1
                    self._silence_count = 0
                else:
                    self._silence_count += 1
                    self._speech_frames.append(frame)  # include trailing silence

                    if self._silence_count >= self._silence_frames:
                        # End of utterance
                        if self._speech_count >= self._min_speech_frames:
                            utterance = np.concatenate(self._speech_frames)
                        self._reset_speech_state()

        return utterance

    def reset(self) -> None:
        """Reset all internal state (call between utterances if needed)."""
        self._buffer = []
        self._energy_history = []
        self._reset_speech_state()

    # ── Internals ─────────────────────────────────────────────────────

    def _reset_speech_state(self) -> None:
        self._in_speech = False
        self._silence_count = 0
        self._speech_count = 0
        self._speech_frames = []

    def _update_energy_history(self, energy: float) -> None:
        self._energy_history.append(energy)
        if len(self._energy_history) > self._history_len:
            self._energy_history.pop(0)

    def _compute_threshold(self) -> float:
        if not self._energy_history:
            return 1e-6
        mean_energy = float(np.mean(self._energy_history))
        return mean_energy * self._energy_factor
