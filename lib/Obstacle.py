#!/usr/bin/env python3
from dataclasses import dataclass


@dataclass
class Obstacle:
    x: float = 0  # task position (meters)
    y: float = 0  # task position (meters)
    r: float = 0