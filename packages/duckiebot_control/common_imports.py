import os
import threading
import time
import numpy as np
import rospy
import cv2
import onnxruntime as ort
import json
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String, Float32
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from typing import Tuple
from duckiebot_control.commands import COMMAND_NAMES, DEFAULT, LEFT, STRAIGHT, RIGHT

NO_ACTION = -1