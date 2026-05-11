from abc import ABC, abstractmethod

import numpy as np


class CameraSource(ABC):
    """Abstract base class for all camera input sources."""

    @abstractmethod
    def get_frame(self) -> 'np.ndarray | None':
        """Return the latest BGR frame as a numpy array, or None if not ready."""

    @abstractmethod
    def is_ready(self) -> bool:
        """Return True when the source has at least one frame available."""
