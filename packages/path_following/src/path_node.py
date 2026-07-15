#!/usr/bin/env python3
from duckiebot_control.onnx_model import OnnxModel, colorize_bev, render_trajectory
from duckiebot_control.node_config import PathConfig
from duckiebot_control.commands import DEFAULT, LEFT, STRAIGHT, RIGHT
from duckiebot_control.controller import Controller
from duckiebot_control.common_imports import *

# Define that the duckiebot currently will not execute any specific action in an intersection
NO_ACTION = -1

class PathNode(DTROS):
    def __init__(self, node_name):
        super(PathNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.config = PathConfig()
        self.running = False
        self.controller = Controller()

        self.model = OnnxModel(os.path.join(self.config.repo_path, "assets", "model.onnx"))

        # Bridge between OpenCV and ROS + window
        self._bridge = CvBridge()
        self._cmdLock = threading.Lock()
        self._goalEpoch = 0

        self.currCommands = []        # commands given by A*
        self.givenCommand = NO_ACTION # command the duckiebot should execute next (fed to the model)
        self._turning = False         # True if the duckiebot is currently "turning" (left, right, straight)

        # Subscriber
        rospy.Subscriber(self.config._cam_topic, CompressedImage, self.cbImage, queue_size=1, buff_size=2**24)
        rospy.Subscriber("/path_commands",Int32MultiArray,self.cbCommands,queue_size=1)
        rospy.Subscriber("/duckiebot_movement",String,self.cbduckiebotCommand,queue_size=1)
        rospy.Subscriber("/heading", Float32MultiArray, self.cbHeadingPidGains, queue_size=1)
        rospy.Subscriber("/velocity", Float32, self.cbVelocity, queue_size=1)

        # Publisher
        self.pubBev         = rospy.Publisher("/bev/compressed", CompressedImage, queue_size=1)
        self.pubTraj        = rospy.Publisher("/trajectory/compressed", CompressedImage, queue_size=1)
        self.pub_car_cmd    = rospy.Publisher(  self.config.cmd_topic, Twist2DStamped, queue_size=1,    
                                                dt_topic_type=TopicType.CONTROL)


    def cbCommands(self, msg):
        """Sets a new list of commands if a new goal has been chosen"""
        with self._cmdLock:
            # Reset the current command structur
            self.currCommands = list(msg.data)
            # We always add an extra command at the end of the command list to ensure our
            # duckiebot drives to the actual goal. Otherwise it would stop just before its last turn.
            self.currCommands.append(DEFAULT)
            # No command popped yet
            self.givenCommand = NO_ACTION

            self._turning = False
            self._goalEpoch += 1


    def popCommand(self):
        """Take the next command off the queue"""

        if self.currCommands:
            self.givenCommand = self.currCommands.pop(0)
            return

        # The queue is drained, so the extra default command we appended was the last one and
        # the goal is behind us. Stop the duckiebot!
        self.givenCommand = NO_ACTION
        self.running = False


    def sequenceCommands(self, action):
        """Check if the current command has successfully been executed"""

        # duckiebot shouldn't turn anyway
        if self.givenCommand in (NO_ACTION, DEFAULT):
            return

        # If duckiebot is not turning, check what he is currently doing
        if not self._turning:
            # Get highest highest score
            top_turn = max((LEFT, STRAIGHT, RIGHT), key=lambda c: action[c])
            # Check if his action represents the given command and if the score is high enough
            if top_turn == self.givenCommand and action[self.givenCommand] >= self.config.spike_threshold:
                self._turning = True
            return
        # If the duckiebot has "turned", the next actions should include a default action
        # Ensures we do not pop commands too early
        if action[DEFAULT] >= self.config.turn_done_threshold:
            self.popCommand()
            self._turning = False

    def cbImage(self, msg):
        
        # Throttle how many times the model computes the waypoints
        # 30 Hz are not possible
        if not self.throttleExecution():
            return
        
        cv_image = self._bridge.compressed_imgmsg_to_cv2(msg)

        with self._cmdLock:
            # At the start (also when we choose another goal) we pop the first command. An empty
            # queue here is not a completed goal, but rather the duckiebot starting without a goal
            if self.givenCommand == NO_ACTION and self.currCommands:
                self.popCommand()

            epoch = self._goalEpoch
            # For the actual model we need to give a number between 0 and 3.
            # The -1 is an information in the code, that no actual command is available.
            if self.givenCommand ==  NO_ACTION:
                cmd = DEFAULT
            else: 
                cmd = self.givenCommand

        # Get the waypoints and current action scores
        _, spatial, temporal, bev_idx, action, _ = self.model.predict(cv_image, cmd)
        rospy.logwarn(f"My actions curr pred: {np.round(action,2)}")

        with self._cmdLock:
            # A new goal landed while we were inside the model. This prediction is conditioned on
            # the command of the old goal, so it must not affect the new goal.
            if epoch != self._goalEpoch:
                return

            # Check if a new command needs to be popped
            self.sequenceCommands(action)

            rospy.logwarn(f"cmd: {self.givenCommand} queue: {self.currCommands} "
                          f"action: {np.round(action, 2)}")

        # This is the model's own prediction, not the pending command
        direction = int(np.argmax(action))
        # Set the parameters for the controller depending on the action
        self.controller.setLookahead(direction)
        self.controller.givenCommand = self.givenCommand
        self.controller.currDirection = direction

        # Publish all gathered information, including which turns, commands, paths etc. for the controller
        self.publishInformation(spatial, temporal, bev_idx, direction)

    def cbduckiebotCommand(self, msg):
        """Using the website we can control if the duckiebot should go or stop"""
        cmd = msg.data.strip()
        if cmd == "start":
            self.running = True
        elif cmd == "stop":
            self.running = False
            self.publishCmd(0.0, 0.0)

    def toPath(self, pts):
        """Validate a model prediction"""

        pts = np.asarray(pts, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[1] != 2 or np.any(np.isnan(pts)):
            rospy.logerr(f"Invalid shape or NaN in waypoints: {pts.shape}")
            return None

        return pts.tolist()

    def publishCmd(self, v, omega):

        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.now()

        car_control_msg.v = v
        car_control_msg.omega = omega

        self.pub_car_cmd.publish(car_control_msg)

    def cbHeadingPidGains(self, msg):
        # Per-action heading tuning from the web UI: [k_tangent, k_offset] per action, in COMMAND
        # order (default, left, straight, right).
        data = list(msg.data)
        if len(data) < 8:
            rospy.logwarn(f"Ignoring heading gains: expected 8 values, got {len(data)}")
            return

        k_tangent_by_action     = {}
        k_offset_by_action = {}
        for action_id in (DEFAULT, LEFT, STRAIGHT, RIGHT):
            base = action_id * 2
            k_tangent_by_action[action_id]     = float(data[base])
            k_offset_by_action[action_id] = float(data[base + 1])

        self.controller.setActionGains(k_tangent_by_action, k_offset_by_action)

        rospy.logwarn(f"Heading tuning updated (per action): k_tangent={k_tangent_by_action} "
                      f"k_offset={k_offset_by_action}")

    def cbVelocity(self, msg):
        self.controller.param.velocity = float(msg.data)
        rospy.logwarn(f"velocity updated: {self.controller.param.velocity}")

    def throttleExecution(self):
        """Skip frames arriving less than one period after the last inference"""
        now = time.monotonic()
        if now - self.config._last_infer < self.config._infer_period:
            return False
        self.config._last_infer = now
        return True

    def publishInformation(self,spatial,temporal,bev_idx,command):
        """Publish the duckiebots velocity and angular velocity and all necessary images"""
        
        
        
        bev_img = colorize_bev(bev_idx)
        self.pubBev.publish(self._bridge.cv2_to_compressed_imgmsg(bev_img))

        # Mark the waypoint the controller is actually chasing.
        target_index = min(self.controller.param.index, len(spatial) - 1)
        spa_img = render_trajectory(spatial=spatial, temporal=temporal, command=command, target_index=target_index)

        self.pubTraj.publish(self._bridge.cv2_to_compressed_imgmsg(spa_img))

        head = self.toPath(spatial)
        velo = self.toPath(temporal)

        # Hand both paths over to the duckiebot together
        self.controller.setPaths(velo, head)


        # As long as the duckiebot should not drive, don't change anything
        if self.running and self.controller.headPath:
            v, omega = self.controller.compute_control()
            if v == 0.0:
                self.publishCmd(0.0, 0.0)
                # If he should stop and there are no commands left, he reached his goal
                if self.currCommands == DEFAULT:
                    self.currCommands = NO_ACTION
                    self.running = False
                # Otherwise he saw an object on his path
            else:
                self.publishCmd(v, omega)
        else:
            self.publishCmd(0.0, 0.0)

if __name__ == "__main__":
    node = PathNode(node_name="path_node")
    rospy.spin()