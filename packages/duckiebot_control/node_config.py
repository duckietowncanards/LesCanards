#!/usr/bin/env python3
import os
class PathConfig:
    def __init__(self):
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._infer_period = 1.0 / 3.0
        self._last_infer = 0.0

        # Necessary spike on the turn action (left,right,straight) 
        self.spike_threshold = 0.5

        # Necessary spike on the default action (after turn action) 
        self.turn_done_threshold = 0.8
        
        self.repo_path = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))))
        
        self._cam_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.veh = os.environ['VEHICLE_NAME']
        self.cmd_topic = f"/{self.veh}/joy_mapper_node/car_cmd"