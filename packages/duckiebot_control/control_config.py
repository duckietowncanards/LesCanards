#!/usr/bin/env python3
from duckiebot_control.common_imports import *

class ControllerParamConfig:
    def __init__(self):
        self.v_min = 0.0
        self.v_max = 0.5
        self.velocity = 0.2
        self.unit_scale = 0.2
        self.index = 2

        # Pure-pursuit lookahead distance (metres, same units as the scaled path).
        # Bigger = smoother/wider turns, smaller = tighter/more aggressive.
        self.lookahead = 0.5

        # k_tangent multiplies the heading-error angle (radians) to the tangent
        self.k_tangent_by_action    = {DEFAULT: 1.0, LEFT: 1.0, STRAIGHT: 1.0, RIGHT: 1.0}

        # Lateral-offset per action.
        self.k_offset_by_action = {DEFAULT: 1.0, LEFT: 0.0, STRAIGHT: 1.0, RIGHT: 0.0}