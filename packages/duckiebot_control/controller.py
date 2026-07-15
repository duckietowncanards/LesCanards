#!/usr/bin/env python3
from duckiebot_control.control_config import ControllerParamConfig
from duckiebot_control.common_imports import *

class Controller():
    """Turns the model's predicted paths into (v, omega)"""

    def __init__(self):

        self.param = ControllerParamConfig()
        self.k_tangent = 0
        self.k_offset = 0

        self.veloPath = []
        self.headPath = []
        self.givenCommand = -1
        self.currDirection = DEFAULT


    def setPaths(self, veloPath, headPath):
        """Install fresh paths."""
        dist_scale = self.param.unit_scale

        self.veloPath = [(dist_scale * x, dist_scale * y) for x,y in veloPath]
        self.headPath = [(dist_scale * x, dist_scale * y) for x,y in headPath]

    def setLookahead(self, action_id):
        """ Pick the heading values for the action the model says it is executing.
        """
        self.k_tangent = self.param.k_tangent_by_action.get(action_id, 2.5)
        self.k_offset = self.param.k_offset_by_action.get(action_id, 0.0)


    def setActionGains(self, k_tangent_by_action, k_offset_by_action):
        """Replace the per-action heading tables live from the web UI."""

        self.param.k_tangent_by_action = k_tangent_by_action
        self.param.k_offset_by_action = k_offset_by_action

    def compute_control(self) -> Tuple[float, float]:
        
        # Compute best index depending on the curve
        self.param.index = self.computeBestIndex()
        
        # Compute constant velocity, might be 0.0 for obstacles
        v = self.computeVelocity()

        # Compute lookahead angle, tangent angle and distance to lookahead point
        alpha,theta, dist = self.computeAnglesValues()
        offset = self.computeOffset()

        omega = v * (2*np.sin(alpha)/dist) + self.k_tangent * theta + self.k_offset * offset


        rospy.logwarn(
            f"[hd] v={v:.3f} k_tangent={self.k_tangent:.2f} omega={omega:.3f} "
            f"idx={self.param.index} n={len(self.headPath)} "
            f"pts={[ (round(x,2), round(y,2)) for x, y in self.headPath[:self.param.index + 2] ]}")

        return (v, omega)


    def computeBestIndex(self):
        """Calculate the point furthest away from the tangent between duckiebot and last point"""

        x1, y1 = self.headPath[-1]

        a = y1
        b = - x1

        best_index = 2
        max_dist = -float("inf")

        for i in range(1, len(self.headPath) - 1):
            x, y = self.headPath[i]
            dist = abs(a * x + b * y) / np.sqrt(a**2 + b**2)

            if dist > max_dist:
                max_dist = dist
                best_index = i

        return best_index

    def computeAnglesValues(self) -> float:
        """Get the angles and distances to a point/tangent depending on the index"""
        
        x0, y0 = self.headPath[self.param.index]
        x1, y1 = self.headPath[self.param.index + 1]
        dx, dy = x1 - x0, y1 - y0
        target_x, target_y = x0, y0

        dist = np.sqrt(target_x**2 + target_y**2)

        path_heading = np.arctan2(-dy, dx)
        alpha = np.arctan2(-target_y,target_x)

        return alpha,path_heading,dist


    def computeOffset(self) -> float:
        """Signed lateral offset of the look-ahead waypoint = how far off the lane center we are."""
        
        # Look at the first point
        _, y_first = self.headPath[0]

        return float(-y_first)


    def computeVelocity(self) -> float:
        
        x,y = self.veloPath[-1]
        dist = np.sqrt(x**2 + y**2)
        
        # If the velocity waypoints collapsed and the duckiebot is on a default route, there is an obstacle
        if dist <= 0.4 and self.currDirection == DEFAULT:
            rospy.logwarn("You have to stop!")
            return 0.0
        
        # If the velocity waypoints almost collpased and the given command is set to default,
        # the duckiebot reached it's goal
        if dist <= 0.8 and self.givenCommand == DEFAULT:
            rospy.logwarn("You reached your goal!")
            return 0.0
        
        v = self.param.velocity

        return float(np.clip(v, self.param.v_min, self.param.v_max))
