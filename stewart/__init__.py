"""6-RSS Stewart platform design toolkit.

Millimetres and radians internally; degrees only when printing a summary or
building a servo command.  Every anchor array is ``(3, 6)`` with column ``i`` =
leg ``i``, and rotations act from the left: ``R @ p``.
"""
from __future__ import annotations

from .geometry import (
    Geometry,
    base_ring,
    make_geometry,
    platform_ring,
    smoke_geometry,
)
from .kinematics import Unreachable, arm_tips, fk, ik, legs, stage1

__version__ = "0.0.1"

__all__ = [
    "Geometry",
    "base_ring",
    "platform_ring",
    "make_geometry",
    "smoke_geometry",
    "Unreachable",
    "stage1",
    "legs",
    "arm_tips",
    "ik",
    "fk",
]
