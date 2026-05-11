"""Pallet detector using HSV colour segmentation."""

from typing import Optional

import cv2
import numpy as np


class PalletDetector:
    """Detects a pallet in a BGR frame via HSV colour thresholding.

    The pipeline is:
    1. Convert BGR → HSV.
    2. Apply ``cv2.inRange`` with the provided HSV bounds.
    3. Morphological open (removes noise) then close (fills holes).
    4. Find external contours and pick the largest one above *min_area*.
    5. Compute bounding rect and centroid.

    Args:
        hsv_lower: Lower HSV bound as a 3-element array-like ``[H, S, V]``.
        hsv_upper: Upper HSV bound as a 3-element array-like ``[H, S, V]``.
        min_area:  Minimum contour area in pixels² to be considered a pallet.
    """

    # Structuring element sizes for morphological operations
    _OPEN_KERNEL_SIZE = (5, 5)
    _CLOSE_KERNEL_SIZE = (9, 9)

    def __init__(
        self,
        hsv_lower: 'list[int] | np.ndarray',
        hsv_upper: 'list[int] | np.ndarray',
        min_area: float = 500.0,
    ) -> None:
        self._hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self._hsv_upper = np.array(hsv_upper, dtype=np.uint8)
        self._min_area = float(min_area)

        self._open_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, self._OPEN_KERNEL_SIZE
        )
        self._close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, self._CLOSE_KERNEL_SIZE
        )

    def detect(self, frame: np.ndarray) -> 'Optional[dict]':
        """Detect the largest pallet in *frame*.

        Args:
            frame: BGR image as a ``np.ndarray`` of shape ``(H, W, 3)``.

        Returns:
            A dict with keys:

            - ``centroid`` – ``(cx, cy)`` pixel coordinates (int tuple).
            - ``bbox``     – ``(x, y, w, h)`` bounding rectangle (int tuple).
            - ``area``     – contour area in pixels² (float).

            Returns ``None`` when no pallet is found.
        """
        if frame is None or frame.size == 0:
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Handle red hue wrap-around (hue ≈ 0° and ≈ 180° are both red)
        if self._hsv_lower[0] <= self._hsv_upper[0]:
            mask = cv2.inRange(hsv, self._hsv_lower, self._hsv_upper)
        else:
            # Wrap-around: e.g. lower=[170,…] upper=[10,…]
            lower_a = np.array([self._hsv_lower[0], self._hsv_lower[1], self._hsv_lower[2]])
            upper_a = np.array([180,               self._hsv_upper[1], self._hsv_upper[2]])
            lower_b = np.array([0,                  self._hsv_lower[1], self._hsv_lower[2]])
            upper_b = np.array([self._hsv_upper[0], self._hsv_upper[1], self._hsv_upper[2]])
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, lower_a, upper_a),
                cv2.inRange(hsv, lower_b, upper_b),
            )

        # Morphological cleaning
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._open_kernel,  iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._close_kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < self._min_area:
            return None

        x, y, w, h = cv2.boundingRect(largest)
        cx = x + w // 2
        cy = y + h // 2

        return {
            'centroid': (cx, cy),
            'bbox':     (x, y, w, h),
            'area':     area,
        }

    @staticmethod
    def draw_detection(frame: np.ndarray, detection: dict) -> np.ndarray:
        """Draw the bounding box and centroid on a copy of *frame*.

        Args:
            frame:     BGR source image.
            detection: Dict returned by :meth:`detect`.

        Returns:
            Annotated BGR image (same dimensions as *frame*).
        """
        annotated = frame.copy()
        x, y, w, h = detection['bbox']
        cx, cy     = detection['centroid']
        area       = detection['area']

        # Bounding rectangle
        cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Centroid cross-hair
        cv2.drawMarker(
            annotated, (cx, cy), (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=20, thickness=2,
        )

        # Label
        label = f'pallet  area={area:.0f}'
        cv2.putText(
            annotated, label, (x, max(y - 6, 14)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
        )
        return annotated
