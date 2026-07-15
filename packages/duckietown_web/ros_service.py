import threading

import cv2
import numpy as np
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

pub = None
pubPath = None
pubHeading = None
pubVScale = None
latest_frame = None
latest_path_points = []
latest_bev = None
latest_trajectory = None

def image_callback(msg):
    global latest_frame

    np_arr = np.frombuffer(msg.data, np.uint8)
    latest_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

def path_points_callback(msg):
    global latest_path_points

    latest_path_points = list(msg.data)

def bev_callback(msg):
    global latest_bev
    latest_bev = bytes(msg.data)

def trajectory_callback(msg):
    global latest_trajectory
    latest_trajectory = bytes(msg.data)



def start_ros_camera():

    rospy.Subscriber(
        "/canards/camera_node/image/compressed",
        CompressedImage,
        image_callback
    )

    rospy.Subscriber(
        "/path_points",
        Int32MultiArray,
        path_points_callback,
        queue_size=10
    )

    rospy.Subscriber(
        "/bev/compressed",
        CompressedImage,
        bev_callback,
        queue_size=1
    )

    rospy.Subscriber(
        "/trajectory/compressed",
        CompressedImage,
        trajectory_callback,
        queue_size=1
    )

    rospy.spin()

def startup_ros():
    global pub, pubPath, pubHeading, pubVScale

    if not rospy.core.is_initialized():
        rospy.init_node(
            "fastapi_node",
            anonymous=False,
            disable_signals=True
        )

    pub = rospy.Publisher("/duckiebot_movement", String, queue_size=10)
    pubPath = rospy.Publisher("/desiredPath", String, queue_size=10)
    pubHeading = rospy.Publisher("/heading", Float32MultiArray, queue_size=1, latch=True)
    pubVScale = rospy.Publisher("/velocity", Float32, queue_size=1, latch=True)

    threading.Thread(
        target=start_ros_camera,
        daemon=True
    ).start()

def publish_route(source, end):

    msg = String()
    msg.data = f"{source},{end}"

    pubPath.publish(msg)

def publish_command(cmd):

    msg = String()
    msg.data = cmd

    pub.publish(msg)

HEADING_PID_ACTIONS = ["default", "left", "straight", "right"]

def publish_heading_pid(actions):

    data = []
    for name in HEADING_PID_ACTIONS:
        g = actions[name]
        data.extend([float(g.k_tangent), float(g.k_offset)])

    msg = Float32MultiArray()
    msg.data = data

    pubHeading.publish(msg)

def publish_velocity(velocity):

    msg = Float32()
    msg.data = float(velocity)

    pubVScale.publish(msg)