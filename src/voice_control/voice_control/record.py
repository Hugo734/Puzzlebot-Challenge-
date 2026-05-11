"""Audio recording module using PyAudio.

Gracefully handles environments where pyaudio is not installed by logging
a warning and raising RuntimeError only when recording is actually attempted.
"""

import logging

import numpy as np

logger = logging.getLogger(__name__)

try:
    import pyaudio
    _PYAUDIO_AVAILABLE = True
except ImportError:
    _PYAUDIO_AVAILABLE = False
    logger.warning(
        'pyaudio is not installed.  AudioRecorder will raise RuntimeError '
        'when recording is attempted.  Install with: pip install pyaudio'
    )


class AudioRecorder:
    """Wraps PyAudio for real-time microphone capture.

    Args:
        sample_rate: Sampling frequency in Hz (default 16 000).
        chunk_size:  Number of frames per read call (default 512).
        channels:    Number of audio channels (default 1 — mono).
    """

    def __init__(
        self,
        sample_rate: int = 16000,
        chunk_size: int = 512,
        channels: int = 1,
    ) -> None:
        self._sample_rate = sample_rate
        self._chunk_size = chunk_size
        self._channels = channels
        self._pa: 'pyaudio.PyAudio | None' = None
        self._stream: 'pyaudio.Stream | None' = None

    # ── Public API ────────────────────────────────────────────────────

    def start(self) -> None:
        """Open the PyAudio input stream."""
        self._assert_pyaudio()
        self._pa = pyaudio.PyAudio()
        self._stream = self._pa.open(
            format=pyaudio.paInt16,
            channels=self._channels,
            rate=self._sample_rate,
            input=True,
            frames_per_buffer=self._chunk_size,
        )
        logger.info(
            'AudioRecorder started — %d Hz, %d ch, chunk=%d',
            self._sample_rate,
            self._channels,
            self._chunk_size,
        )

    def stop(self) -> None:
        """Close the PyAudio input stream and terminate PyAudio."""
        if self._stream is not None:
            self._stream.stop_stream()
            self._stream.close()
            self._stream = None
        if self._pa is not None:
            self._pa.terminate()
            self._pa = None
        logger.info('AudioRecorder stopped')

    def read_chunk(self) -> np.ndarray:
        """Read one chunk of audio.

        Returns:
            float32 array of shape (chunk_size,), normalised to [-1, 1].

        Raises:
            RuntimeError: If the stream has not been started.
        """
        if self._stream is None:
            raise RuntimeError('AudioRecorder.start() must be called before read_chunk()')

        raw = self._stream.read(self._chunk_size, exception_on_overflow=False)
        samples = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
        return samples / 32768.0

    def record_segment(self, duration_s: float) -> np.ndarray:
        """Blocking capture of a fixed-duration audio segment.

        Args:
            duration_s: Duration in seconds.

        Returns:
            float32 array of shape (n_samples,), normalised to [-1, 1].
        """
        self._assert_pyaudio()
        n_chunks = int(np.ceil(duration_s * self._sample_rate / self._chunk_size))

        was_running = self._stream is not None
        if not was_running:
            self.start()

        chunks: list[np.ndarray] = []
        for _ in range(n_chunks):
            chunks.append(self.read_chunk())

        if not was_running:
            self.stop()

        return np.concatenate(chunks)

    # ── Helpers ───────────────────────────────────────────────────────

    @staticmethod
    def _assert_pyaudio() -> None:
        if not _PYAUDIO_AVAILABLE:
            raise RuntimeError(
                'pyaudio is not installed.  Run: pip install pyaudio'
            )
