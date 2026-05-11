"""Trailer / truck loading-area detector using classical CV.

Detection strategy (no trained model required):
1. Convert to HSV and threshold for yellow/orange truck markings.
2. Apply morphological cleaning.
3. Find external contours; filter by area and approximate as quadrilaterals.
4. Pick the best candidate (largest area, most rectangular shape).
5. Compute centroid and a simple confidence score based on how close the
   approximated polygon is to a rectangle (4 corners, aspect ratio).

A second strategy (large rectangular contour in any colour) is used as
fallback when the HSV thresholding finds nothing.
"""

from typing import Optional

import cv2
import numpy as np


# Default HSV range for yellow/orange truck markings
_YELLOW_LOWER = np.array([18,  80, 80],  dtype=np.uint8)
_YELLOW_UPPER = np.array([35, 255, 255], dtype=np.uint8)
_ORANGE_LOWER = np.array([10,  80, 80],  dtype=np.uint8)
_ORANGE_UPPER = np.array([18, 255, 255], dtype=np.uint8)

# Polygon approximation epsilon as fraction of arc length
_POLY_EPSILON_RATIO = 0.04
# Minimum aspect-ratio width/height for a "wide" loading bay
_MIN_ASPECT = 0.8
_MAX_ASPECT = 4.0


def _contour_to_detection(contour: np.ndarray) -> Optional[dict]:
    """Convert a contour to a detection dict, or None if not rectangular."""
    epsilon = _POLY_EPSILON_RATIO * cv2.arcLength(contour, closed=True)
    approx = cv2.approxPolyDP(contour, epsilon, closed=True)
    if len(approx) < 4 or len(approx) > 6:
        return None

    x, y, w, h = cv2.boundingRect(contour)
    if h == 0:
        return None
    aspect = w / h
    if aspect < _MIN_ASPECT or aspect > _MAX_ASPECT:
        return None

    area = float(cv2.contourArea(contour))
    cx = x + w // 2
    cy = y + h // 2

    # Confidence: closer to 4 corners and good aspect → higher score
    corner_score = 1.0 - abs(len(approx) - 4) / 4.0
    aspect_score = 1.0 - abs(aspect - 1.5) / max(abs(_MAX_ASPECT - 1.5), 1.0)
    confidence = float(np.clip((corner_score + aspect_score) / 2.0, 0.0, 1.0))

    return {
        'bbox':       (int(x), int(y), int(w), int(h)),
        'centroid':   (int(cx), int(cy)),
        'area':       area,
        'confidence': confidence,
    }


class TrailerDetector:
    """Detect the truck trailer loading area in a BGR frame.

    Args:
        min_area: Minimum contour area in pixels² to be considered valid.
    """

    _OPEN_KERNEL  = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    _CLOSE_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))

    def __init__(self, min_area: float = 2000.0) -> None:
        self._min_area = float(min_area)

    def detect(self, frame: np.ndarray) -> Optional[dict]:
        """Detect the trailer/truck loading area.

        Args:
            frame: BGR image.

        Returns:
            A dict with keys ``bbox`` ``(x,y,w,h)``, ``centroid`` ``(cx,cy)``,
            and ``confidence`` (0–1), or ``None`` when nothing is found.
        """
        if frame is None or frame.size == 0:
            return None

        result = self._detect_by_colour(frame)
        if result is None:
            result = self._detect_by_large_rectangle(frame)
        return result

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _colour_mask(self, frame: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_yellow = cv2.inRange(hsv, _YELLOW_LOWER, _YELLOW_UPPER)
        mask_orange = cv2.inRange(hsv, _ORANGE_LOWER, _ORANGE_UPPER)
        mask = cv2.bitwise_or(mask_yellow, mask_orange)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._OPEN_KERNEL,  iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._CLOSE_KERNEL, iterations=2)
        return mask

    def _detect_by_colour(self, frame: np.ndarray) -> Optional[dict]:
        mask = self._colour_mask(frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = [
            det
            for c in contours
            if cv2.contourArea(c) >= self._min_area
            for det in [_contour_to_detection(c)]
            if det is not None
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda d: d['area'] * d['confidence'])

    def _detect_by_large_rectangle(self, frame: np.ndarray) -> Optional[dict]:
        """Fallback: look for the largest rectangular contour in the edge image."""
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur  = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = [
            det
            for c in contours
            if cv2.contourArea(c) >= self._min_area * 2
            for det in [_contour_to_detection(c)]
            if det is not None
        ]
        if not candidates:
            return None
        best = max(candidates, key=lambda d: d['area'])
        # Lower confidence for edge-only detection
        best['confidence'] = best['confidence'] * 0.5
        return best

    @staticmethod
    def draw_detection(frame: np.ndarray, detection: dict) -> np.ndarray:
        """Draw the detected trailer bounding box on a copy of *frame*.

        Args:
            frame:     BGR source image.
            detection: Dict returned by :meth:`detect`.

        Returns:
            Annotated BGR image.
        """
        annotated = frame.copy()
        x, y, w, h = detection['bbox']
        cx, cy     = detection['centroid']
        conf       = detection['confidence']

        cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 165, 255), 3)
        cv2.drawMarker(
            annotated, (cx, cy), (255, 0, 0),
            markerType=cv2.MARKER_DIAMOND,
            markerSize=18, thickness=2,
        )
        cv2.putText(
            annotated,
            f'trailer  conf={conf:.2f}',
            (x, max(y - 6, 14)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2,
        )
        return annotated
