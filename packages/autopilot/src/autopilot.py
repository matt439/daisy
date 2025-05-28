#!/usr/bin/env python3

import rospy
import math
from duckietown_msgs.msg import Twist2DStamped, FSMState, AprilTagDetectionArray
from std_msgs.msg import Float64, Int8
from enum import Enum
from abc import ABC, abstractmethod

AUTOPILOT_UPDATE_FREQUENCY = 20  # Hz
STOP_SIGN_WAITING_TIME = 5.0  # seconds
LANE_FOLLOWING_STOP_SIGN_TIME = 3.0  # seconds
STOPPING_DISTANCE_TARGET = 0.1  # meters
STOP_SIGN_IDS = [1, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38]
LANE_FOLLOWING_FSM_STATE = "LANE_FOLLOWING"
NORMAL_JOYSTICK_CONTROL_FSM_STATE = "NORMAL_JOYSTICK_CONTROL"

LANE_CONTROLLER_NODE_V_BAR = "/vader/lane_controller_node/v_bar" # nominal velocity in m/s
LANE_CONTROLLER_NODE_K_D = "/vader/lane_controller_node/k_d" # proportional term for lateral deviation
LANE_CONTROLLER_NODE_K_THETA = "/vader/lane_controller_node/k_theta" # proportional term for heading deviation
LANE_CONTROLLER_NODE_K_ID = "/vader/lane_controller_node/k_Id" # integral term for lateral deviation
LANE_CONTROLLER_NODE_K_IPHI = "/vader/lane_controller_node/k_IphI" # integral term for lateral deviation?
LANE_CONTROLLER_NODE_THETA_THRES_MIN = "/vader/lane_controller_node/theta_thres_min" # minimum value for heading error
LANE_CONTROLLER_NODE_THETA_THRES_MAX = "/vader/lane_controller_node/theta_thres_max" # maximum value for heading error

V_BAR = 0.3 # clamped from 0 to 5, default 0.19
K_D = -30.0 # clamped from -100 to 100, default -6.0
K_THETA = -9.0 # clamped from -100 to 100, default -5.0
K_ID = -0.3 # clamped from -100 to 100, default -0.3
K_IPHI = 0.0 # clamped from -100 to 100, default 0.0
THETA_THRES_MIN = -0.5 # clamped from -100 to 100, default -0.5
THETA_THRES_MAX = 0.75 # clamped from -100 to 100, default 0.75

class DuckieBotEvent(Enum):
    STOP_SIGN_DETECTED = 0
    BOT_BECOMES_STOPPED = 1
    MOVEMENT_CONTROLLER_SUCCEEDED = 6
    MOVEMENT_CONTROLLER_FAILED = 7

class Timer:
    def __init__(self, duration: float):
        self._duration = duration
        self._start_time = None
    def start(self):
        self._start_time = rospy.get_time()
    def is_expired(self) -> bool:
        if self._start_time is None:
            return False
        return (rospy.get_time() - self._start_time) >= self._duration
    def reset(self):
        self._start_time = None

class Duckiebot():
    def __init__(self, state: 'DuckiebotState', state_pub, goal_dist_pub, goal_angle_pub) -> None:
        self._state_publisher = state_pub
        self._goal_distance_publisher = goal_dist_pub
        self._goal_angle_publisher = goal_angle_pub
        rospy.loginfo("Duckiebot class initialized!")
        self.transition_to(state)

    def transition_to(self, state: 'DuckiebotState'):
        self._state = state
        self._state._context = self
        rospy.loginfo(f"Transitioning to state: {type(state).__name__}")
        self._state.on_enter()  # Call on_enter after context is set

    def on_event(self, event: DuckieBotEvent) -> None:
        rospy.loginfo(f"Event received: {event}")
        self._state.on_event(event)

    def update(self):
        self._state.update()

    def publish_FSM_state(self, state: str):
        fsm_state_msg = FSMState()
        fsm_state_msg.header.stamp = rospy.Time.now()
        fsm_state_msg.state = state
        self._state_publisher.publish(fsm_state_msg)

    def publish_goal_distance(self, distance: float):
        goal_distance_msg = Float64()
        goal_distance_msg.data = distance
        self._goal_distance_publisher.publish(goal_distance_msg)

    def publish_goal_angle(self, angle: float):
        goal_angle_msg = Float64()
        goal_angle_msg.header.stamp = rospy.Time.now()
        goal_angle_msg.data = angle
        self._goal_angle_publisher.publish(goal_angle_msg)

    def stop_bot(self):
        self.publish_goal_distance(STOPPING_DISTANCE_TARGET)

class DuckiebotState(ABC):
    @property
    def context(self) -> Duckiebot:
        return self._context
    
    @context.setter
    def context(self, context: Duckiebot) -> None:
        self._context = context

    def on_enter(self):
        pass

    @abstractmethod
    def on_event(self, event: DuckieBotEvent) -> None:
        pass

    @abstractmethod
    def update(self) -> None:
        pass

class LaneFollowingState(DuckiebotState):
    def __init__(self):
        pass

    def on_enter(self):
        self._context.publish_FSM_state(LANE_FOLLOWING_FSM_STATE)

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.STOP_SIGN_DETECTED:
            self.context.transition_to(StoppingForStopSignState())

    def update(self) -> None:
        pass

class StoppingForStopSignState(DuckiebotState):
    def __init__(self):
        pass

    def on_enter(self):
        self._context.publish_FSM_state(NORMAL_JOYSTICK_CONTROL_FSM_STATE) # Stop lane following
        self._context.stop_bot() # Stop the robot

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.BOT_BECOMES_STOPPED:
            self.context.transition_to(WaitingAtStopSignState())

    def update(self) -> None:
        pass

class WaitingAtStopSignState(DuckiebotState):
    def __init__(self):
        self._timer = Timer(STOP_SIGN_WAITING_TIME)

    def on_enter(self):
        self._timer.start()

    def on_event(self, event: DuckieBotEvent) -> None:
        pass

    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(LaneFollowingStopSignState())

class LaneFollowingStopSignState(DuckiebotState):
    def __init__(self):
        self._timer = Timer(LANE_FOLLOWING_STOP_SIGN_TIME)

    def on_enter(self):
        self._context.publish_FSM_state(LANE_FOLLOWING_FSM_STATE)
        self._timer.start()

    # No event handling for stop sign detection in this state
    def on_event(self, event: DuckieBotEvent) -> None:
        pass

    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(LaneFollowingState())

class Autopilot:
    def __init__(self):
        #Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        self.cmd_vel_pub = rospy.Publisher('/vader/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self._state_publisher = rospy.Publisher('/vader/fsm_node/mode', FSMState, queue_size=1)
        self._goal_distance_publisher = rospy.Publisher('/vader/goal_distance', Float64, queue_size=1)
        self._goal_angle_publisher = rospy.Publisher('/vader/goal_angle', Float64, queue_size=1)
        rospy.Subscriber('/vader/fsm_node/mode', FSMState, self.FSM_state_callback, queue_size=1)
        rospy.Subscriber('/vader/apriltag_detector_node/detections',
                         AprilTagDetectionArray, self.april_tag_callback, queue_size=1)

        self.set_lane_following_parameters()

        self._duckiebot = Duckiebot(LaneFollowingState(), self._state_publisher,
                                    self._goal_distance_publisher, self._goal_angle_publisher)

        rospy.loginfo("Initialized autopilot node!")

    def FSM_state_callback(self, msg: FSMState):
        rospy.loginfo(f"FSM state changed to: {msg.state}")
        if msg.state == 'MOVEMENT_CONTROLLER_SUCCESS':
            self._duckiebot.on_event(DuckieBotEvent.MOVEMENT_CONTROLLER_SUCCEEDED)
            self._duckiebot.on_event(DuckieBotEvent.BOT_BECOMES_STOPPED)
        elif msg.state == 'MOVEMENT_CONTROLLER_FAILURE':
            self._duckiebot.on_event(DuckieBotEvent.MOVEMENT_CONTROLLER_FAILED)

    def april_tag_callback(self, msg: AprilTagDetectionArray):
        # Process the AprilTag detections
        for detection in msg.detections:
            id = detection.tag_id
            if self.is_stop_sign_id(id):
                self._duckiebot.on_event(DuckieBotEvent.STOP_SIGN_DETECTED)
            else:
                rospy.loginfo(f"Unknown tag ID: {id}")
    
    def set_lane_following_parameters(self):
        rospy.set_param(LANE_CONTROLLER_NODE_V_BAR, V_BAR)
        rospy.set_param(LANE_CONTROLLER_NODE_K_D, K_D)
        rospy.set_param(LANE_CONTROLLER_NODE_K_THETA, K_THETA)
        rospy.set_param(LANE_CONTROLLER_NODE_K_ID, K_ID)
        rospy.set_param(LANE_CONTROLLER_NODE_K_IPHI, K_IPHI)
        rospy.set_param(LANE_CONTROLLER_NODE_THETA_THRES_MIN, THETA_THRES_MIN)
        rospy.set_param(LANE_CONTROLLER_NODE_THETA_THRES_MAX, THETA_THRES_MAX)
        rospy.loginfo("Lane following parameters set!")

    # Stop Robot before node has shut down. This ensures the robot keep moving with the latest velocity command
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def is_stop_sign_id(self, tag_id):
        return tag_id in STOP_SIGN_IDS
    
    def run(self):
        rate = rospy.Rate(AUTOPILOT_UPDATE_FREQUENCY)
        while not rospy.is_shutdown():
            self._duckiebot.update()
            rate.sleep()

if __name__ == '__main__':
    try:
        autopilot = Autopilot()
        autopilot.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
