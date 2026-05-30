"""Scribit hardware constants and default configuration."""
from __future__ import annotations

import math
from typing import Dict

# Default distance between nails (mm)
D_MM_DEFAULT = 1860

# Pen slot Z angles (degrees)
PEN_SLOTS_Z: Dict[int, int] = {1: 89, 2: 161, 3: 233, 4: 305}

# Known Z reference after homing (G77)
Z_AFTER_G77 = -56.0

# Default starting position on the wall
_ = math.sqrt(1240**2 - 1000**2)
STARTING_X, STARTING_Y = (1000, _)
