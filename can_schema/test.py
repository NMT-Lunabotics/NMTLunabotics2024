#!/usr/bin/env python3

"""Test of can_schema."""

from can_schema import (
    Database,
    Message,
    Signal,
)

Database.with_messages([
    Message('left_motor', 100, [
        Signal.from_labels([
            'stop',
            'forward',
            'reverse',
        ]),
    ]),
    Message('right_motor', 101, [
        Signal.from_labels([
            'stop',
            'forward',
            'reverse',
        ]),
    ]),
]).instantiate()
