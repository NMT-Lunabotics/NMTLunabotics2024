#!/usr/bin/env python3

"""CAN schema library.

Uses Python as an eDSL for laying out CAN frame data types, like DBC
and KCD. Generates C++ code for encoding and decoding.

"""

from typing import List


class Database:
    """Set of CAN frames that can be decoded."""
    def __init__(self):
        pass

    def with_messages(messages: List[Message]) -> Database:
        """Create a new database with the given messages."""
        pass

    def instantiate(self) -> str:
        """Generate C++ code."""
        pass


class Message:
    """An interpretation of a single CAN ID number's data."""
    def __init__(
            self,
            name: str,
            id_num: int,
            signals: List['Signal'],
    ):
        pass

    def instantiate(self) -> str:
        """Generate C++ code."""
        pass


class Signal:
    """A CAN signal."""
    def __init__(self):
        pass

    @staticmethod
    def from_labels(labels: List[str]) -> Signal:
        """A signal with the given possible labels, like an enum."""
        pass

    @staticmethod
    def with_range(width: int, minimum: float, maximum: float) -> Signal:
        """A signal with the given bit width, minimum, and maximum."""
        pass

    @staticmethod
    def skip(width: int) -> Signal:
        """Generate a fake signal that's just ignored data in the message."""
        pass

    def instantiate(self) -> str:
        """Generate C++ code."""
        pass
