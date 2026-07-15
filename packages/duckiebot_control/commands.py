#!/usr/bin/env python3
"""Single source of truth for the navigation command ids shared by the model and the planner.

COMMAND_NAMES is the model's output-class order"""

COMMAND_NAMES = ["default", "left", "straight", "right"]
DEFAULT = COMMAND_NAMES.index("default")
LEFT = COMMAND_NAMES.index("left")
STRAIGHT = COMMAND_NAMES.index("straight")
RIGHT = COMMAND_NAMES.index("right")
