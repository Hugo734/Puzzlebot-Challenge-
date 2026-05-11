from abc import ABC, abstractmethod


class GpioDriver(ABC):
    """Abstract base class for GPIO drivers controlling the FPGA lifter."""

    @abstractmethod
    def setup(self) -> None:
        """Initialize GPIO hardware."""

    @abstractmethod
    def set_level(self, level: int) -> None:
        """Set lifter to level 0-7 via 3-bit binary encoding.

        Bit mapping (board pin numbering):
            bit 0 → pin 11
            bit 1 → pin 13
            bit 2 → pin 15

        Args:
            level: Integer in range [0, 7].
        """

    @abstractmethod
    def cleanup(self) -> None:
        """Release GPIO resources."""
