"""Mock camera source that synthesises BGR frames for simulation and unit tests."""

from typing import Optional

import numpy as np

from .base import CameraSource

# Named colours in BGR
_COLOUR_MAP: dict[str, tuple[int, int, int]] = {
    'red':    (0,   0,   200),
    'green':  (0,   200, 0),
    'blue':   (200, 0,   0),
    'yellow': (0,   200, 200),
    'orange': (0,   140, 255),
    'white':  (255, 255, 255),
}

_PALLET_W = 80   # pixel width of the synthetic pallet rectangle
_PALLET_H = 50   # pixel height


class MockCameraSource(CameraSource):
    """Returns synthetic test frames.

    The frame contains a plain grey background with one coloured rectangle
    that represents the pallet.  The rectangle's centroid can be moved via
    :meth:`set_pallet_position` to exercise the alignment PID controller in
    simulation without real hardware.
    """

    def __init__(self, width: int = 640, height: int = 480) -> None:
        self._width = width
        self._height = height
        # Centroid starts at image centre
        self._cx: int = width // 2
        self._cy: int = height // 2
        self._colour: tuple[int, int, int] = _COLOUR_MAP['red']

    # ------------------------------------------------------------------
    # CameraSource interface
    # ------------------------------------------------------------------

    def get_frame(self) -> np.ndarray:
        """Always returns a freshly rendered BGR frame."""
        frame = np.full((self._height, self._width, 3), 60, dtype=np.uint8)

        # Draw simulated pallet as a filled rectangle
        x1 = max(0, self._cx - _PALLET_W // 2)
        y1 = max(0, self._cy - _PALLET_H // 2)
        x2 = min(self._width - 1,  self._cx + _PALLET_W // 2)
        y2 = min(self._height - 1, self._cy + _PALLET_H // 2)
        frame[y1:y2, x1:x2] = self._colour

        # Draw white crosshair at the image centre for visual reference
        cx_img = self._width // 2
        cy_img = self._height // 2
        frame[cy_img - 10:cy_img + 10, cx_img] = (255, 255, 255)
        frame[cy_img, cx_img - 10:cx_img + 10] = (255, 255, 255)

        return frame

    def is_ready(self) -> bool:
        """Mock source is always ready."""
        return True

    # ------------------------------------------------------------------
    # Test helper
    # ------------------------------------------------------------------

    def set_pallet_position(
        self,
        cx: int,
        cy: int,
        color: str = 'red',
    ) -> None:
        """Move the simulated pallet centroid.

        Args:
            cx: Horizontal pixel position of the pallet centroid.
            cy: Vertical pixel position of the pallet centroid.
            color: One of ``red``, ``green``, ``blue``, ``yellow``,
                   ``orange``, or ``white``.
        """
        self._cx = int(cx)
        self._cy = int(cy)
        self._colour = _COLOUR_MAP.get(color.lower(), _COLOUR_MAP['red'])
