"""ArUco marker detector compatible with OpenCV 4.x and 4.7+."""

import math
from typing import Optional

import cv2
import numpy as np

from geometry_msgs.msg import Pose


def _rotation_matrix_to_quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a quaternion (x, y, z, w)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


def _rvec_tvec_to_pose(rvec: np.ndarray, tvec: np.ndarray) -> Pose:
    """Convert OpenCV rvec/tvec to a geometry_msgs/Pose."""
    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    qx, qy, qz, qw = _rotation_matrix_to_quaternion(R)

    pose = Pose()
    t = tvec.flatten()
    pose.position.x    = float(t[0])
    pose.position.y    = float(t[1])
    pose.position.z    = float(t[2])
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


# ---------------------------------------------------------------------------
# Detector API wrapper — handles the cv2.aruco API break in OpenCV 4.7
# ---------------------------------------------------------------------------

def _build_aruco_detector(dict_id: int) -> object:
    """Return an object with a ``detectMarkers(frame)`` method.

    Tries the new ``cv2.aruco.ArucoDetector`` API (OpenCV >= 4.7) first,
    then falls back to the legacy ``cv2.aruco.detectMarkers``.
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

    if hasattr(cv2.aruco, 'ArucoDetector'):
        # OpenCV >= 4.7
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)

        class _NewAPIWrapper:
            def detectMarkers(self, frame: np.ndarray):  # noqa: N802
                corners, ids, rejected = detector.detectMarkers(frame)
                return corners, ids, rejected

        return _NewAPIWrapper()
    else:
        # OpenCV < 4.7 legacy API
        params = cv2.aruco.DetectorParameters_create()  # type: ignore[attr-defined]

        class _OldAPIWrapper:
            def __init__(self, d, p):
                self._d = d
                self._p = p

            def detectMarkers(self, frame: np.ndarray):  # noqa: N802
                corners, ids, rejected = cv2.aruco.detectMarkers(  # type: ignore[attr-defined]
                    frame, self._d, parameters=self._p
                )
                return corners, ids, rejected

        return _OldAPIWrapper(aruco_dict, params)


class ArucoDetector:
    """Detects ArUco markers and estimates their 6-DOF pose.

    Args:
        camera_matrix:  3x3 intrinsic matrix as a ``np.ndarray``.
        dist_coeffs:    Distortion coefficients (1x5 or similar).
        marker_length:  Physical side length of the marker in metres.
        dict_id:        ArUco dictionary identifier
                        (default ``cv2.aruco.DICT_4X4_50``).
    """

    def __init__(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        marker_length: float = 0.05,
        dict_id: int = cv2.aruco.DICT_4X4_50,
    ) -> None:
        self._camera_matrix = np.array(camera_matrix, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs   = np.array(dist_coeffs,   dtype=np.float64).flatten()
        self._marker_length = float(marker_length)
        self._detector = _build_aruco_detector(dict_id)

    def detect(self, frame: np.ndarray) -> 'list[dict]':
        """Detect all ArUco markers in *frame* and estimate their poses.

        Args:
            frame: BGR image.

        Returns:
            A list of dicts (one per detected marker), each containing:

            - ``id``      – marker integer ID.
            - ``corners`` – ``np.ndarray`` of shape ``(4, 2)`` image corners.
            - ``rvec``    – rotation vector ``np.ndarray`` shape ``(3,)``.
            - ``tvec``    – translation vector ``np.ndarray`` shape ``(3,)``.
            - ``pose``    – ``geometry_msgs/Pose`` in camera frame.
        """
        if frame is None or frame.size == 0:
            return []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            return []

        results: list[dict] = []
        for i, marker_id in enumerate(ids.flatten()):
            marker_corners = corners[i].reshape(4, 2)

            # estimatePoseSingleMarkers expects shape (N, 1, 4, 2)
            corners_input = corners[i].reshape(1, 1, 4, 2)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners_input,
                self._marker_length,
                self._camera_matrix,
                self._dist_coeffs,
            )
            rvec = rvecs[0].flatten()
            tvec = tvecs[0].flatten()

            pose = _rvec_tvec_to_pose(rvec, tvec)

            results.append({
                'id':      int(marker_id),
                'corners': marker_corners,
                'rvec':    rvec,
                'tvec':    tvec,
                'pose':    pose,
            })

        return results

    def draw_detections(self, frame: np.ndarray, detections: 'list[dict]') -> np.ndarray:
        """Draw detected markers and their axes on a copy of *frame*.

        Args:
            frame:      BGR source image.
            detections: List returned by :meth:`detect`.

        Returns:
            Annotated BGR image.
        """
        annotated = frame.copy()
        for det in detections:
            corners_draw = det['corners'].reshape(1, 4, 2).astype(np.float32)
            cv2.aruco.drawDetectedMarkers(annotated, [corners_draw])
            try:
                cv2.drawFrameAxes(
                    annotated,
                    self._camera_matrix,
                    self._dist_coeffs,
                    det['rvec'],
                    det['tvec'],
                    self._marker_length * 0.5,
                )
            except cv2.error:
                # drawFrameAxes not available on very old builds
                pass
            cx = int(det['corners'][:, 0].mean())
            cy = int(det['corners'][:, 1].mean())
            cv2.putText(
                annotated,
                f'id={det["id"]}',
                (cx - 10, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2,
            )
        return annotated
