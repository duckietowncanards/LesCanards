#!/usr/bin/env python3
from graph_computation.a_star import A_Star
from graph_computation.town_graph import Graph
from duckiebot_control.common_imports import *

class PlanningNode(DTROS):
    def __init__(self, node_name):
        super(PlanningNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.GENERIC
        )
        self.a_star = A_Star(Graph())

        # Subscriber
        rospy.Subscriber("/desiredPath",String,self.cbComputePath,queue_size=10)
        # Publisher
        self.pointPub = rospy.Publisher("/path_points",Int32MultiArray,queue_size=10)
        self.commandPub = rospy.Publisher("/path_commands",Int32MultiArray,queue_size=10)

    def cbComputePath(self, msg):
        source, end = map(int, msg.data.split(","))
        if source == -1 or end == -1:
            self.pointPub.publish([])
            return
        
        points, commands = self.a_star.compute_a_star_msg(source, end)
        self.pointPub.publish(points)
        self.commandPub.publish(commands)
        rospy.logwarn(f"CMD: {points}")
        rospy.logwarn(f"CMD: {commands}")

if __name__ == "__main__":
    node = PlanningNode(node_name="path_planning_node")
    rospy.spin()
