#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import FSMState, AprilTagDetectionArray, \
                                AprilTagDetection, WheelsCmdStamped
from std_msgs.msg import Int8, Float64MultiArray
from enum import Enum
from abc import ABC, abstractmethod
from typing import Tuple
import random
import math

# Autopilot node constants
AUTOPILOT_UPDATE_FREQUENCY = 20  # Hz
WHEEL_VELOCITY_STOPPED_THRESHOLD = 0.01  # m/s, threshold to consider the wheel stopped

# FSM states
LANE_FOLLOWING_FSM_STATE = "LANE_FOLLOWING"
NORMAL_JOYSTICK_CONTROL_FSM_STATE = "NORMAL_JOYSTICK_CONTROL"

# Sign constants
APPROACHING_SIGN_SLOWDOWN_DISTANCE = 0.1  # meters, distance at which the bot starts slowing down
APPROACHING_SIGN_SLOWDOWN_DURATION = 1.0  # seconds, duration of the slowdown phase
SIGN_DETECTION_DISTANCE_THRESHOLD = 0.4  # meters, distance at which the bot detects the sign
APPROACHING_SIGN_TIMEOUT_DURATION = 7.0  # seconds
SIGN_WAITING_DURATION = 2.0  # seconds
# APRIL_TAG_DETCTION_ROTATION_THRESHOLD = 0.5  # Threshold for quaternion components to determine valid tag orientation

# Stop sign constants
LANE_FOLLOWING_STOP_SIGN_TIME = 4.0  # seconds
STOP_SIGN_IDS = [1, 20, 21, 22, 23, 24, 25, 26, 27,
                 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38]

# Overtaking constants
OVERTAKING_MANEUVER_DURATION = 5.0  # seconds, duration of the overtaking maneuver
STOPPING_FOR_CAR_TIMEOUT_DURATION = 3.0  # seconds
CAR_WAITING_TIME = 3.0  # seconds
TRAPEZOIDAL_RULE_N = 100  # Number of intervals for trapezoidal rule integration
OVERTAKING_MAX_VELOCITY = 0.8  # m/s, maximum velocity during overtaking
OVERTAKING_MIN_VELOCITY = 0.0  # m/s, minimum velocity during overtaking
OVERTAKING_FORWARD_DISTANCE = 0.5  # meters
OVERTAKING_MIDWAY_DISTANCE = 0.25 # meters, where the piecewise function is split into two parts 
OVERTAKING_WHEEL_OFFSET = 0.09  # meters
# Generalized logistic function parameters for overtaking
A = 0.0
K = 0.25
B = 70.0
X0 = 0.08
V = 1.0

# Intersection constants
LEFT_INTERSECTION_SIGNS_IDS = [10, 61, 62, 63, 64]
RIGHT_INTERSECTION_SIGNS_IDS = [9, 57, 58, 59, 60]
T_INTERSECTION_SIGNS_IDS = [11, 65, 66, 67, 68]

# Turning constants
TURN_MAX_VELOCITY_FACTOR = 2.0
TURN_MIN_VELOCITY_FACTOR = 0.0
AXLE_LENGTH = 0.1  # meters, distance between the two wheels

TURN_LEFT_RIGHT_WHEEL_RADIUS = 0.39  # meters, radius of the right wheel during left turn
TURN_LEFT_LEFT_WHEEL_RADIUS = TURN_LEFT_RIGHT_WHEEL_RADIUS - AXLE_LENGTH  # meters, radius of the left wheel during left turn
TURN_LEFT_LEFT_WHEEL_VELOCITY_ADJUSTMENT_SCALAR = 1.0  # scalar to adjust the velocity for left turn
TURN_LEFT_RIGHT_WHEEL_VELOCITY_ADJUSTMENT_SCALAR = 1.0  # scalar to adjust the velocity for right wheel during left turn
TURN_LEFT_LEFT_WHEEL_DISTANCE = math.pi * TURN_LEFT_LEFT_WHEEL_RADIUS * \
    TURN_LEFT_LEFT_WHEEL_VELOCITY_ADJUSTMENT_SCALAR / 2.0  # meters, distance traveled by the left wheel during left turn
TURN_LEFT_RIGHT_WHEEL_DISTANCE = math.pi * TURN_LEFT_RIGHT_WHEEL_RADIUS * \
    TURN_LEFT_RIGHT_WHEEL_VELOCITY_ADJUSTMENT_SCALAR / 2.0  # meters, distance traveled by the right wheel during left turn
TURN_LEFT_MANEUVER_DURATION = 2.7  # seconds
TURN_LEFT_LEFT_WHEEL_VELOCITY = TURN_LEFT_LEFT_WHEEL_DISTANCE / TURN_LEFT_MANEUVER_DURATION  # m/s, velocity of the left wheel during left turn
TURN_LEFT_RIGHT_WHEEL_VELOCITY = TURN_LEFT_RIGHT_WHEEL_DISTANCE / TURN_LEFT_MANEUVER_DURATION  # m/s, velocity of the right wheel during left turn

TURN_RIGHT_LEFT_WHEEL_RADIUS = 0.17  # meters, radius of the left wheel during right turn
TURN_RIGHT_RIGHT_WHEEL_RADIUS = TURN_RIGHT_LEFT_WHEEL_RADIUS - AXLE_LENGTH  # meters, radius of the right wheel during right turn
TURN_RIGHT_LEFT_WHEEL_VELOCITY_ADJUSTMENT_SCALAR = 1.0  # scalar to adjust the velocity for left wheel during right turn
TURN_RIGHT_RIGHT_WHEEL_VELOCITY_ADJUSTMENT_SCALAR = 1.0  # scalar to adjust the velocity for right wheel during right turn
TURN_RIGHT_LEFT_WHEEL_DISTANCE = math.pi * TURN_RIGHT_LEFT_WHEEL_RADIUS * \
    TURN_RIGHT_LEFT_WHEEL_VELOCITY_ADJUSTMENT_SCALAR / 2.0  # meters, distance traveled by the left wheel during right turn
TURN_RIGHT_RIGHT_WHEEL_DISTANCE = math.pi * TURN_RIGHT_RIGHT_WHEEL_RADIUS * \
    TURN_RIGHT_RIGHT_WHEEL_VELOCITY_ADJUSTMENT_SCALAR / 2.0  # meters, distance traveled by the right wheel during right turn
TURN_RIGHT_MANEUVER_DURATION = 1.2  # seconds
TURN_RIGHT_LEFT_WHEEL_VELOCITY = TURN_RIGHT_LEFT_WHEEL_DISTANCE / TURN_RIGHT_MANEUVER_DURATION  # m/s, velocity of the left wheel during right turn
TURN_RIGHT_RIGHT_WHEEL_VELOCITY = TURN_RIGHT_RIGHT_WHEEL_DISTANCE / TURN_RIGHT_MANEUVER_DURATION  # m/s, velocity of the right wheel during right turn

# Lane controller node parameters constants
LANE_CONTROLLER_NODE_V_BAR = "/vader/lane_controller_node/v_bar" # nominal velocity in m/s
LANE_CONTROLLER_NODE_K_D = "/vader/lane_controller_node/k_d" # proportional term for lateral deviation
LANE_CONTROLLER_NODE_K_THETA = "/vader/lane_controller_node/k_theta" # proportional term for heading deviation
LANE_CONTROLLER_NODE_K_ID = "/vader/lane_controller_node/k_Id" # integral term for lateral deviation
LANE_CONTROLLER_NODE_K_IPHI = "/vader/lane_controller_node/k_IphI" # integral term for lateral deviation?
LANE_CONTROLLER_NODE_THETA_THRES_MIN = "/vader/lane_controller_node/theta_thres_min" # minimum value for heading error
LANE_CONTROLLER_NODE_THETA_THRES_MAX = "/vader/lane_controller_node/theta_thres_max" # maximum value for heading error
V_BAR = 0.25 # clamped from 0 to 5, default 0.19
K_D = -25.0 # clamped from -100 to 100, default -6.0
K_THETA = -7.0 # clamped from -100 to 100, default -5.0
K_ID = -0.3 # clamped from -100 to 100, default -0.3
K_IPHI = 0.0 # clamped from -100 to 100, default 0.0
THETA_THRES_MIN = -0.5 # clamped from -100 to 100, default -0.5
THETA_THRES_MAX = 0.75 # clamped from -100 to 100, default 0.75

# Lane controller node parameters which cannot be set dynamically:
#
# d_thres (:obj:`float`): Maximum value for lateral error
# d_offset (:obj:`float`): Goal offset from center of the lane
# integral_bounds (:obj:`dict`): Bounds for integral term
# d_resolution (:obj:`float`): Resolution of lateral position estimate
# phi_resolution (:obj:`float`): Resolution of heading estimate
# omega_ff (:obj:`float`): Feedforward part of controller
# verbose (:obj:`bool`): Verbosity level (0,1,2)
# stop_line_slowdown (:obj:`dict`): Start and end distances for slowdown at stop lines

class DuckieBotEvent(Enum):
    STOP_SIGN_DETECTED = 0
    BOT_BECOMES_STOPPED = 1
    CAR_DETECTED = 2
    CAR_REMOVED = 3
    TURN_LEFT_SIGN_DETECTED = 4
    TURN_RIGHT_SIGN_DETECTED = 5
    T_INTERSECTON_SIGN_DETECTED = 6
    WHEEL_MOVEMENT_INFO_RECEIVED = 7
    PAUSE_COMMAND_RECEIVED = 8
    RESUME_COMMAND_RECEIVED = 9

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
    def get_elapsed_time(self) -> float:
        if self._start_time is None:
            return 0.0
        return rospy.get_time() - self._start_time

class WheelMovementInfo:
    def __init__(self):
        self._left_distance = 0.0
        self._left_displacement = 0.0
        self._left_velocity = 0.0
        self._right_distance = 0.0
        self._right_displacement = 0.0
        self._right_velocity = 0.0
        # self._last_update_time = rospy.Time.now()

    def update(self, msg: Float64MultiArray):
        if len(msg.data) != 6:
            rospy.logwarn("Invalid wheel movement info message received, expected six elements.")
            return
        self._left_distance = msg.data[0]
        self._left_displacement = msg.data[1]
        self._left_velocity = msg.data[2]
        self._right_distance = msg.data[3]
        self._right_displacement = msg.data[4]
        self._right_velocity = msg.data[5]
        # self._last_update_time = rospy.Time.now()

    # def get_left_info(self) -> float:
    #     return (self._left_distance, self._left_displacement, self._left_velocity)
    def get_left_distance(self) -> float:
        return self._left_distance
    def get_left_displacement(self) -> float:
        return self._left_displacement
    def get_left_velocity(self) -> float:
        return self._left_velocity
    # def get_right_info(self) -> float:
    #     return (self._right_distance, self._right_displacement, self._right_velocity)
    def get_right_distance(self) -> float:
        return self._right_distance
    def get_right_displacement(self) -> float:
        return self._right_displacement
    def get_right_velocity(self) -> float:
        return self._right_velocity
    # def get_last_update_time(self):
    #     return self._last_update_time
    def get_average_velocity(self) -> float:
        return (self._left_velocity + self._right_velocity) / 2.0
    
    # def print_info(self):
    #     rospy.loginfo(f"Left - Dist: {self._left_distance}, Disp: {self._left_displacement}, Vel: {self._left_velocity}")
    #     rospy.loginfo(f"Right - Dist: {self._right_distance}, Disp: {self._right_displacement}, Vel: {self._right_velocity}")

class AprilTagTools:
    @staticmethod
    def is_stop_sign_id(tag_id) -> bool:
        return tag_id in STOP_SIGN_IDS
    
    @staticmethod
    def is_left_intersection_sign_id(tag_id) -> bool:
        return tag_id in LEFT_INTERSECTION_SIGNS_IDS
    
    @staticmethod
    def is_right_intersection_sign_id(tag_id) -> bool:
        return tag_id in RIGHT_INTERSECTION_SIGNS_IDS
    
    @staticmethod
    def is_t_intersection_sign_id(tag_id) -> bool:
        return tag_id in T_INTERSECTION_SIGNS_IDS
    
    @staticmethod
    def is_april_tag_in_valid_position(detection : AprilTagDetection) -> bool:
        if detection.transform.translation.x < 0.0:
            # Check if the tag is to the left of the bot
            return False
        
        if detection.transform.translation.z > SIGN_DETECTION_DISTANCE_THRESHOLD:
            # The tag is too far away, ignore it
            return False

        # # Check if the tag is not at an extreme angle using the x, y, z components of the quaternion
        # # w is not used for angle checks, so we can ignore it
        # if abs(detection.transform.rotation.x) > APRIL_TAG_DETCTION_ROTATION_THRESHOLD or \
        #         abs(detection.transform.rotation.y) > APRIL_TAG_DETCTION_ROTATION_THRESHOLD or \
        #             abs(detection.transform.rotation.z) > APRIL_TAG_DETCTION_ROTATION_THRESHOLD:
        #     return False
        
        # If all checks passed, the tag is in a valid position
        return True
    
    @staticmethod
    def print_april_tag_info(detection: AprilTagDetection):
        rospy.loginfo(f"AprilTag ID: {detection.tag_id}")
        rospy.loginfo(f"Position: x={detection.transform.translation.x}, "
                      f"y={detection.transform.translation.y}, "
                      f"z={detection.transform.translation.z}")
        rospy.loginfo(f"Rotation: x={detection.transform.rotation.x}, "
                      f"y={detection.transform.rotation.y}, "
                      f"z={detection.transform.rotation.z}, "
                      f"w={detection.transform.rotation.w}")
    
    # @staticmethod
    # def is_sign_in_stoppable_position(detection: AprilTagDetection) -> bool:
    #     z = detection.transform.translation.z
    #     if z < FOLLOW_Z_DISTANCE_TARGET + APPROACHING_SIGN_SLOWDOWN_DISTANCE:
    #         # There is not enough distance to stop before the sign
    #         return False
    #     return True

class Duckiebot():
    def __init__(self, state: 'DuckiebotState', state_pub) -> None:
        self._sign_tag_id = None
        self._wheel_movement_info = WheelMovementInfo()
        self._most_recent_april_tag = None
        rospy.Subscriber('/vader/wheel_movement_info', Float64MultiArray, self.wheel_movement_info_callback, queue_size=1)
        rospy.Subscriber('/vader/apriltag_detector_node/detections', AprilTagDetectionArray, self.april_tag_callback, queue_size=1)
        self._state_publisher = state_pub
        self._velocity_publisher = rospy.Publisher('/vader/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        # self._cmd_vel_publisher = rospy.Publisher('/vader/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.loginfo("Duckiebot class initialized!")
        self.transition_to(state)

    def transition_to(self, state: 'DuckiebotState'):
        self._state = state
        self._state._context = self
        rospy.loginfo(f"Transitioning to state: {type(state).__name__}")
        self._state.on_enter()  # Call on_enter after context is set

    def on_event(self, event: DuckieBotEvent) -> None:
        # if event is not DuckieBotEvent.TURN_LEFT_SIGN_DETECTED and \
        #     event is not DuckieBotEvent.TURN_RIGHT_SIGN_DETECTED and \
        #     event is not DuckieBotEvent.T_INTERSECTON_SIGN_DETECTED and \
        #     event is not DuckieBotEvent.STOP_SIGN_DETECTED and \
        #     event is not DuckieBotEvent.BOT_BECOMES_STOPPED and \
        #     event is not DuckieBotEvent.WHEEL_MOVEMENT_INFO_RECEIVED:
        #     # Avoid logging too many events
        #     rospy.loginfo(f"Event received: {event}")
        self._state.on_event(event)

    def update(self):
        self._state.update()

    def publish_FSM_state(self, state: str):
        fsm_state_msg = FSMState()
        fsm_state_msg.header.stamp = rospy.Time.now()
        fsm_state_msg.state = state
        self._state_publisher.publish(fsm_state_msg)

    # def publish_cmd_vel(self, v: float, omega: float):
    #     cmd_msg = Twist2DStamped()
    #     cmd_msg.header.stamp = rospy.Time.now()
    #     cmd_msg.v = v
    #     cmd_msg.omega = omega
    #     self._cmd_vel_publisher.publish(cmd_msg)

    def publish_velocity(self, left_velocity: float, right_velocity: float):
        wheels_cmd = WheelsCmdStamped()
        wheels_cmd.header.stamp = rospy.Time.now()
        wheels_cmd.vel_left = left_velocity
        wheels_cmd.vel_right = right_velocity
        self._velocity_publisher.publish(wheels_cmd)

    def stop_bot(self):
        self.publish_FSM_state(NORMAL_JOYSTICK_CONTROL_FSM_STATE)  # Stop lane following
        self.publish_velocity(0.0, 0.0)
    
    def wheel_movement_info_callback(self, msg: Float64MultiArray):
        self._wheel_movement_info.update(msg)
        # self._wheel_movement_info.print_info()
        if self.is_wheels_stopped():
            self.on_event(DuckieBotEvent.BOT_BECOMES_STOPPED)
        self.on_event(DuckieBotEvent.WHEEL_MOVEMENT_INFO_RECEIVED)

    def get_wheel_movement_info(self) -> WheelMovementInfo:
        return self._wheel_movement_info
    
    def is_wheels_stopped(self) -> bool:
        left_velocity = self._wheel_movement_info.get_left_velocity()
        right_velocity = self._wheel_movement_info.get_right_velocity()
        return abs(left_velocity) < WHEEL_VELOCITY_STOPPED_THRESHOLD and \
               abs(right_velocity) < WHEEL_VELOCITY_STOPPED_THRESHOLD
    
    def get_most_recent_april_tag(self) -> AprilTagDetection:
        return self._most_recent_april_tag
    
    def april_tag_callback(self, msg: AprilTagDetectionArray):
        for detection in msg.detections:
            if not AprilTagTools.is_april_tag_in_valid_position(detection):
                continue

            id = detection.tag_id
            self._most_recent_april_tag = detection  # Update the most recent tag
            if AprilTagTools.is_stop_sign_id(id):
                self.on_event(DuckieBotEvent.STOP_SIGN_DETECTED)
            elif AprilTagTools.is_left_intersection_sign_id(id):
                self.on_event(DuckieBotEvent.TURN_LEFT_SIGN_DETECTED)
            elif AprilTagTools.is_right_intersection_sign_id(id):
                self.on_event(DuckieBotEvent.TURN_RIGHT_SIGN_DETECTED)
            elif AprilTagTools.is_t_intersection_sign_id(id):
                self.on_event(DuckieBotEvent.T_INTERSECTON_SIGN_DETECTED)
            else:
                rospy.loginfo(f"Unknown tag ID: {id}")

class DuckiebotState(ABC):
    @property
    def context(self) -> Duckiebot:
        return self._context
    
    @context.setter
    def context(self, context: Duckiebot) -> None:
        self._context = context

    @abstractmethod
    def on_enter(self) -> None:
        pass

    @abstractmethod
    def on_event(self, event: DuckieBotEvent) -> None:
        pass

    @abstractmethod
    def update(self) -> None:
        pass

class PauseState(DuckiebotState):
    def __init__(self):
        pass

    def on_enter(self) -> None:
        self.context.publish_FSM_state('PAUSED')

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.RESUME_COMMAND_RECEIVED:
            self.context.transition_to(LaneFollowingState())

    def update(self) -> None:
        pass

class LaneFollowingState(DuckiebotState):
    def __init__(self):
        pass

    def on_enter(self) -> None:
        self.context.publish_FSM_state(LANE_FOLLOWING_FSM_STATE)

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.STOP_SIGN_DETECTED:
            self.context.transition_to(StoppingForStopSignState(self.context.get_most_recent_april_tag()))
        elif event == DuckieBotEvent.CAR_DETECTED:
            self.context.transition_to(StoppingForCarState())
        elif event == DuckieBotEvent.TURN_LEFT_SIGN_DETECTED:
            self.context.transition_to(ApproachingTurnLeftSignState(self.context.get_most_recent_april_tag()))
        elif event == DuckieBotEvent.TURN_RIGHT_SIGN_DETECTED:
            self.context.transition_to(ApproachingTurnRightSignState(self.context.get_most_recent_april_tag()))
        elif event == DuckieBotEvent.T_INTERSECTON_SIGN_DETECTED:
            tag = self.context.get_most_recent_april_tag()
            # Randomly choose between left and right turn for T-intersection
            if random.choice([True, False]):
                self.context.transition_to(ApproachingTurnLeftSignState(tag))
            else:
                self.context.transition_to(ApproachingTurnRightSignState(tag))

    def update(self) -> None:
        pass

class ApproachingSignTools:
    @staticmethod
    def calculate_slow_down_veloticy(total_duration: float, current_duration: float,
                                     start_velocity: float, end_velocity: float) -> float:
        if current_duration >= total_duration:
            return end_velocity
        # Calculate the velocity based on the elapsed time
        return start_velocity + (end_velocity - start_velocity) * (current_duration / total_duration)
    
    @staticmethod
    def is_at_slowdown_position(wheel_info: WheelMovementInfo, detection_left_distance: float,
                                detection_right_distance: float, detection_z_distance: float) -> bool:
        left_distance = wheel_info.get_left_distance()
        right_distance = wheel_info.get_right_distance()

        target_left_distance = detection_left_distance + detection_z_distance - APPROACHING_SIGN_SLOWDOWN_DISTANCE
        target_right_distance = detection_right_distance + detection_z_distance - APPROACHING_SIGN_SLOWDOWN_DISTANCE

        # rospy.loginfo(f"Left distance: {left_distance}, Target left distance: {target_left_distance}")
        # rospy.loginfo(f"Right distance: {right_distance}, Target right distance: {target_right_distance}")
        if left_distance >= target_left_distance and right_distance >= target_right_distance:
            return True
        return False
    
    @staticmethod
    def calculate_slowdown_velocity(slowdown_timer: Timer, start_slowdown_velocity: float) -> float:
        return ApproachingSignTools.calculate_slow_down_veloticy(
            APPROACHING_SIGN_SLOWDOWN_DURATION,
            slowdown_timer.get_elapsed_time(),
            start_slowdown_velocity,
            0.0)

class StoppingForStopSignState(DuckiebotState):
    def __init__(self, tag: AprilTagDetection):
        self._timer = Timer(APPROACHING_SIGN_TIMEOUT_DURATION)
        self._slowdown_timer = Timer(APPROACHING_SIGN_SLOWDOWN_DURATION)
        self._in_slowdown_phase = False
        self._start_slowdown_velocity = math.inf
        self._detection_left_distance = math.inf
        self._detection_right_distance = math.inf
        self._detection_z_distance = tag.transform.translation.z

    def on_enter(self) -> None:
        self._timer.start()
        self.context.publish_FSM_state(LANE_FOLLOWING_FSM_STATE)
        wheel_info = self.context.get_wheel_movement_info()
        self._detection_left_distance = wheel_info.get_left_distance()
        self._detection_right_distance = wheel_info.get_right_distance()

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.WHEEL_MOVEMENT_INFO_RECEIVED:
            if self._in_slowdown_phase:
                vel = ApproachingSignTools.calculate_slowdown_velocity(
                    self._slowdown_timer, self._start_slowdown_velocity)
                self.context.publish_velocity(vel, vel)
                return
            
            if ApproachingSignTools.is_at_slowdown_position(self.context.get_wheel_movement_info(),
                            self._detection_left_distance, self._detection_right_distance,
                                self._detection_z_distance):
                self.context.publish_FSM_state(NORMAL_JOYSTICK_CONTROL_FSM_STATE) # Stop lane following
                self._slowdown_timer.start()
                # Get the average velocity of the wheels at the start of the slowdown phase
                self._start_slowdown_velocity = self.context.get_wheel_movement_info().get_average_velocity()
                self._in_slowdown_phase = True

    def update(self) -> None:
        if self._timer.is_expired():
            rospy.logwarn("Stopping for stop sign timer expired, transitioning to lane following state.")
            self.context.transition_to(LaneFollowingState())
            return

        if self._slowdown_timer.is_expired():
            self.context.stop_bot() # Stop the robot, as we are at the correct position
            self.context.transition_to(WaitingAtStopSignState())

class WaitingAtStopSignState(DuckiebotState):
    def __init__(self):
        self._timer = Timer(SIGN_WAITING_DURATION)

    def on_enter(self) -> None:
        self._timer.start()
        self.context.stop_bot()  # Stop the robot, in case it wasn't already stopped

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())

    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(LaneFollowingStopSignState())

class LaneFollowingStopSignState(DuckiebotState):
    def __init__(self):
        self._timer = Timer(LANE_FOLLOWING_STOP_SIGN_TIME)

    def on_enter(self) -> None:
        self.context.publish_FSM_state(LANE_FOLLOWING_FSM_STATE)
        self._timer.start()

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.CAR_DETECTED:
            self.context.transition_to(StoppingForCarState())
        elif event == DuckieBotEvent.TURN_LEFT_SIGN_DETECTED:
            self.context.transition_to(ApproachingTurnLeftSignState(self.context.get_most_recent_april_tag()))
        elif event == DuckieBotEvent.TURN_RIGHT_SIGN_DETECTED:
            self.context.transition_to(ApproachingTurnRightSignState(self.context.get_most_recent_april_tag()))
        elif event == DuckieBotEvent.T_INTERSECTON_SIGN_DETECTED:
            tag = self.context.get_most_recent_april_tag()
            # Randomly choose between left and right turn for T-intersection
            if random.choice([True, False]):
                self.context.transition_to(ApproachingTurnLeftSignState(tag))
            else:
                self.context.transition_to(ApproachingTurnRightSignState(tag))

    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(LaneFollowingState())
    
class StoppingForCarState(DuckiebotState):
    def __init__(self):
        self._timer = Timer(STOPPING_FOR_CAR_TIMEOUT_DURATION)

    def on_enter(self) -> None:
        self._context.stop_bot() # Stop the robot
        self._timer.start()

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.BOT_BECOMES_STOPPED:
            self.context.transition_to(WaitingForCarState())
        elif event == DuckieBotEvent.CAR_REMOVED:
            self.context.transition_to(LaneFollowingState())

    def update(self) -> None:
        if self._timer.is_expired():
            rospy.logwarn("Stopping for car timer expired, transitioning to lane following state.")
            self.context.transition_to(LaneFollowingState())

class WaitingForCarState(DuckiebotState):
    def __init__(self):
        self._timer = Timer(CAR_WAITING_TIME)

    def on_enter(self) -> None:
        self._timer.start()
        self.context.stop_bot()  # Stop the robot, in case it wasn't already stopped

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.CAR_REMOVED:
            self.context.transition_to(LaneFollowingState())

    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(OvertakingState())

class OvertakingTools:
    @staticmethod
    def logistic_derivative(x: float, A: float, K: float,
                            B: float, x0: float, V: float) -> float:
        exp_term = math.exp(-B * (x - x0))
        return B * V * (K - A) * exp_term / ((1 + exp_term)**(V + 1))

    @staticmethod
    def track_derivative(t: float, A: float, K: float, B: float, x0: float, V: float,
                         midway: float, wheel_offset: float, is_left: bool) -> float:
        if is_left:
            if t < midway:
                return OvertakingTools.logistic_derivative(t, A, K, B, x0, V)
            else:
                return -OvertakingTools.logistic_derivative(t - midway - wheel_offset, A, K, B, x0, V)
        else:
            if t < midway:
                return OvertakingTools.logistic_derivative(t - wheel_offset, A, K, B, x0, V)
            else:
                return -OvertakingTools.logistic_derivative(t - midway, A, K, B, x0, V)

    @staticmethod
    def track_arc_length_integrand(t: float, A: float, K: float, B: float, x0: float, V: float,
                                    midway: float, wheel_offset: float, is_left: bool) -> float:
        # invert is_left as the arc length required is opposite as the bot turns right
        # when the left wheel moves forward and vice versa
        if is_left:
            is_left = False
        else:
            is_left = True
        derivative = OvertakingTools.track_derivative(t, A, K, B, x0, V, midway, wheel_offset, is_left)
        return math.sqrt(1 + derivative ** 2)
    
    @staticmethod
    def trapezoidal_rule(f, a: float, b: float, n: int, args=()) -> float:
        h = (b - a) / n
        total = (f(a, *args) + f(b, *args)) / 2.0
        for i in range(1, n):
            total += f(a + i * h, *args)
        return total * h

    @staticmethod
    def cumulative_track_distance(a: float, b: float, A: float, K: float, B: float, x0: float,
                                  V: float, midway: float, wheel_offset: float, is_left: bool) -> float:
        return OvertakingTools.trapezoidal_rule(
            OvertakingTools.track_arc_length_integrand, a, b, TRAPEZOIDAL_RULE_N,
                args=(A, K, B, x0, V, midway, wheel_offset, is_left))

class VelocityCalculator:
    @staticmethod
    def calculate_velocity(target_distance: float, current_distance: float, time: float,
                           min_vel: float, max_vel: float) -> float:
        if target_distance <= current_distance:
            velocity = 0.0  # No need to move if we are already at or beyond the target distance
        else: # Calculate the velocity needed to reach the target distance in the given time
            velocity = (target_distance - current_distance) / time

        # Clamp the velocity to the maximum and minimum values
        if velocity > max_vel:
            velocity = max_vel
        elif velocity < min_vel:
            velocity = min_vel
        return velocity

class OvertakingState(DuckiebotState):
    def __init__(self):
        self._timer = Timer(OVERTAKING_MANEUVER_DURATION)
        self._start_left_distance = 0.0
        self._start_right_distance = 0.0
        self._last_wheel_info_time = None
    
    def on_enter(self) -> None:
        self._timer.start()
        self._context.publish_FSM_state(NORMAL_JOYSTICK_CONTROL_FSM_STATE)  # Stop lane following
        wheel_info = self.context.get_wheel_movement_info()
        self._start_left_distance = wheel_info.get_left_distance()
        self._start_right_distance = wheel_info.get_right_distance()
        self._last_wheel_info_time = rospy.get_time()

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.WHEEL_MOVEMENT_INFO_RECEIVED:
            if self._last_wheel_info_time is not None:
                self.control_overtaking()

    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(LaneFollowingState())

    def control_overtaking(self):
        wheel_info = self.context.get_wheel_movement_info()
        left_distance = wheel_info.get_left_distance()
        right_distance = wheel_info.get_right_distance()

        # Calculate the current distances from the starting point
        current_left_distance = left_distance - self._start_left_distance
        current_right_distance = right_distance - self._start_right_distance

        timer_elapsed = self._timer.get_elapsed_time()

        # The x-axis of the arc length integral is the forward axis of the bot.
        # Adjust the timer to be a proportion of the total overtaking distance,
        # relative to the elapsed time over the total overtaking maneuver duration.
        adjusted_timer = OVERTAKING_FORWARD_DISTANCE * (timer_elapsed / OVERTAKING_MANEUVER_DURATION)

        target_left_distance = OvertakingTools.cumulative_track_distance(
            0.0, adjusted_timer, A, K, B, X0, V,
                OVERTAKING_MIDWAY_DISTANCE, OVERTAKING_WHEEL_OFFSET, True)
        
        target_right_distance = OvertakingTools.cumulative_track_distance(
            0.0, adjusted_timer, A, K, B, X0, V,
                OVERTAKING_MIDWAY_DISTANCE, OVERTAKING_WHEEL_OFFSET, False)
        
        current_time = rospy.get_time()
        elapsed_time = current_time - self._last_wheel_info_time
        self._last_wheel_info_time = current_time

        # Calculate the velocities for the left and right wheels
        left_velocity = VelocityCalculator.calculate_velocity(
            target_left_distance, current_left_distance, elapsed_time,
                OVERTAKING_MIN_VELOCITY, OVERTAKING_MAX_VELOCITY)
        right_velocity = VelocityCalculator.calculate_velocity(
            target_right_distance, current_right_distance, elapsed_time,
                OVERTAKING_MIN_VELOCITY, OVERTAKING_MAX_VELOCITY)

        # Publish the velocities to the wheels
        self.context.publish_velocity(left_velocity, right_velocity)

class TurningTools:
    @staticmethod
    def calculate_turning_track_distance(is_left_turn: bool, is_left_wheel: bool, timer_elapsed: float) -> float:
        if is_left_turn:
            if is_left_wheel:
                return TURN_LEFT_LEFT_WHEEL_DISTANCE * timer_elapsed / TURN_LEFT_MANEUVER_DURATION
            else: # right wheel
                return TURN_LEFT_RIGHT_WHEEL_DISTANCE * timer_elapsed / TURN_LEFT_MANEUVER_DURATION
        else: # right turn
            if is_left_wheel:
                return TURN_RIGHT_LEFT_WHEEL_DISTANCE * timer_elapsed / TURN_RIGHT_MANEUVER_DURATION
            else: # right wheel
                return TURN_RIGHT_RIGHT_WHEEL_DISTANCE * timer_elapsed / TURN_RIGHT_MANEUVER_DURATION
            
    @staticmethod
    def calculate_min_max_vels(is_left_turn: bool, is_left_wheel: bool) -> Tuple[float, float]:
        if is_left_turn:
            if is_left_wheel:
                min_vel = TURN_LEFT_LEFT_WHEEL_VELOCITY * TURN_MIN_VELOCITY_FACTOR
                max_vel = TURN_LEFT_LEFT_WHEEL_VELOCITY * TURN_MAX_VELOCITY_FACTOR
            else: # right wheel
                min_vel = TURN_LEFT_RIGHT_WHEEL_VELOCITY * TURN_MIN_VELOCITY_FACTOR
                max_vel = TURN_LEFT_RIGHT_WHEEL_VELOCITY * TURN_MAX_VELOCITY_FACTOR
        else:  # right turn
            if is_left_wheel:
                min_vel = TURN_RIGHT_LEFT_WHEEL_VELOCITY * TURN_MIN_VELOCITY_FACTOR
                max_vel = TURN_RIGHT_LEFT_WHEEL_VELOCITY * TURN_MAX_VELOCITY_FACTOR
            else:  # right wheel
                min_vel = TURN_RIGHT_RIGHT_WHEEL_VELOCITY * TURN_MIN_VELOCITY_FACTOR
                max_vel = TURN_RIGHT_RIGHT_WHEEL_VELOCITY * TURN_MAX_VELOCITY_FACTOR
        return (min_vel, max_vel)

class TurningLeftState(DuckiebotState):
    def __init__(self):
        self._timer = Timer(TURN_LEFT_MANEUVER_DURATION)
        self._start_left_distance = 0.0
        self._start_right_distance = 0.0
        self._last_wheel_info_time = None

    def on_enter(self) -> None:
        self._timer.start()
        self.context.stop_bot()  # Stop the robot, in case it wasn't already stopped
        wheel_info = self.context.get_wheel_movement_info()
        self._start_left_distance = wheel_info.get_left_distance()
        self._start_right_distance = wheel_info.get_right_distance()
        self._last_wheel_info_time = rospy.get_time()

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.WHEEL_MOVEMENT_INFO_RECEIVED:
            if self._last_wheel_info_time is not None:
                self.control_turning()

    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(LaneFollowingState())

    def control_turning(self):
        wheel_info = self.context.get_wheel_movement_info()
        left_distance = wheel_info.get_left_distance()
        right_distance = wheel_info.get_right_distance()

        # Calculate the current distances from the starting point
        current_left_distance = left_distance - self._start_left_distance
        current_right_distance = right_distance - self._start_right_distance

        timer_elapsed = self._timer.get_elapsed_time()

        target_left_distance = TurningTools.calculate_turning_track_distance(
            True, True, timer_elapsed)
        
        target_right_distance = TurningTools.calculate_turning_track_distance(
            True, False, timer_elapsed)
        
        current_time = rospy.get_time()
        elapsed_time = current_time - self._last_wheel_info_time
        self._last_wheel_info_time = current_time

        (min_left_vel, max_left_vel) = TurningTools.calculate_min_max_vels(True, True)
        (min_right_vel, max_right_vel) = TurningTools.calculate_min_max_vels(True, False)

        # Calculate the velocities for the left and right wheels
        left_velocity = VelocityCalculator.calculate_velocity(
            target_left_distance, current_left_distance, elapsed_time,
            min_left_vel, max_left_vel)
        right_velocity = VelocityCalculator.calculate_velocity(
            target_right_distance, current_right_distance, elapsed_time,
            min_right_vel, max_right_vel)

        # rospy.loginfo(f"Left velocity: {left_velocity}, Right velocity: {right_velocity}")
        # rospy.loginfo(f"Left min velocity: {min_left_vel}, Left max velocity: {max_left_vel}")
        # rospy.loginfo(f"Right min velocity: {min_right_vel}, Right max velocity: {max_right_vel}")
        # rospy.loginfo(f"timer_elapsed: {timer_elapsed}")
        # rospy.loginfo(f"target_left_distance: {target_left_distance}, target_right_distance: {target_right_distance}")
        # rospy.loginfo(f"current_left_distance: {current_left_distance}, current_right_distance: {current_right_distance}")
        # rospy.loginfo(f"elapsed_time: {elapsed_time}")
        # rospy.loginfo(f"Wheel movement info: Left distance: {left_distance}, Right distance: {right_distance}")
        # rospy.loginfo(f"Start left distance: {self._start_left_distance}, Start right distance: {self._start_right_distance}")
        # rospy.loginfo("")

        # Publish the velocities to the wheels
        self.context.publish_velocity(left_velocity, right_velocity)

class TurningRightState(DuckiebotState):
    def __init__(self):
        self._timer = Timer(TURN_RIGHT_MANEUVER_DURATION)
        self._start_left_distance = 0.0
        self._start_right_distance = 0.0
        self._last_wheel_info_time = None

    def on_enter(self) -> None:
        self._timer.start()
        self.context.stop_bot()  # Stop the robot, in case it wasn't already stopped
        wheel_info = self.context.get_wheel_movement_info()
        self._start_left_distance = wheel_info.get_left_distance()
        self._start_right_distance = wheel_info.get_right_distance()
        self._last_wheel_info_time = rospy.get_time()

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.WHEEL_MOVEMENT_INFO_RECEIVED:
            if self._last_wheel_info_time is not None:
                self.control_turning()

    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(LaneFollowingState())

    def control_turning(self):
        wheel_info = self.context.get_wheel_movement_info()
        left_distance = wheel_info.get_left_distance()
        right_distance = wheel_info.get_right_distance()

        # Calculate the current distances from the starting point
        current_left_distance = left_distance - self._start_left_distance
        current_right_distance = right_distance - self._start_right_distance

        timer_elapsed = self._timer.get_elapsed_time()

        target_left_distance = TurningTools.calculate_turning_track_distance(
            False, True, timer_elapsed)
        
        target_right_distance = TurningTools.calculate_turning_track_distance(
            False, False, timer_elapsed)
        
        current_time = rospy.get_time()
        elapsed_time = current_time - self._last_wheel_info_time
        self._last_wheel_info_time = current_time

        (min_left_vel, max_left_vel) = TurningTools.calculate_min_max_vels(False, True)
        (min_right_vel, max_right_vel) = TurningTools.calculate_min_max_vels(False, False)

        # Calculate the velocities for the left and right wheels
        left_velocity = VelocityCalculator.calculate_velocity(
            target_left_distance, current_left_distance, elapsed_time,
            min_left_vel, max_left_vel)
        right_velocity = VelocityCalculator.calculate_velocity(
            target_right_distance, current_right_distance, elapsed_time,
            min_right_vel, max_right_vel)

        # Publish the velocities to the wheels
        self.context.publish_velocity(left_velocity, right_velocity)

class WaitingAtTurnLeftSignState(DuckiebotState):
    def __init__(self):
        self._timer = Timer(SIGN_WAITING_DURATION)

    def on_enter(self) -> None:
        self._timer.start()
        self.context.stop_bot()  # Stop the robot, in case it wasn't already stopped

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
    
    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(TurningLeftState())

class WaitingAtTurnRightSignState(DuckiebotState):
    def __init__(self):
        self._timer = Timer(SIGN_WAITING_DURATION)

    def on_enter(self) -> None:
        self._timer.start()
        self.context.stop_bot()  # Stop the robot, in case it wasn't already stopped

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
    
    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(TurningRightState())

class ApproachingTurnLeftSignState(DuckiebotState):
    def __init__(self, tag: AprilTagDetection):
        self._timer = Timer(APPROACHING_SIGN_TIMEOUT_DURATION)
        self._slowdown_timer = Timer(APPROACHING_SIGN_SLOWDOWN_DURATION)
        self._in_slowdown_phase = False
        self._start_slowdown_velocity = math.inf
        self._detection_left_distance = math.inf
        self._detection_right_distance = math.inf
        self._detection_z_distance = tag.transform.translation.z

    def on_enter(self) -> None:
        self._timer.start()
        self.context.publish_FSM_state(LANE_FOLLOWING_FSM_STATE)
        wheel_info = self.context.get_wheel_movement_info()
        self._detection_left_distance = wheel_info.get_left_distance()
        self._detection_right_distance = wheel_info.get_right_distance()

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.WHEEL_MOVEMENT_INFO_RECEIVED:
            if self._in_slowdown_phase:
                vel = ApproachingSignTools.calculate_slowdown_velocity(
                    self._slowdown_timer, self._start_slowdown_velocity)
                self.context.publish_velocity(vel, vel)
                return
            
            if ApproachingSignTools.is_at_slowdown_position(self.context.get_wheel_movement_info(),
                            self._detection_left_distance, self._detection_right_distance,
                                self._detection_z_distance):
                self.context.publish_FSM_state(NORMAL_JOYSTICK_CONTROL_FSM_STATE) # Stop lane following
                self._slowdown_timer.start()
                # Get the average velocity of the wheels at the start of the slowdown phase
                self._start_slowdown_velocity = self.context.get_wheel_movement_info().get_average_velocity()
                self._in_slowdown_phase = True

    def update(self) -> None:
        if self._timer.is_expired():
            rospy.logwarn("Approaching turn left sign timer expired, transitioning to lane following state.")
            self.context.transition_to(LaneFollowingState())
            return

        if self._slowdown_timer.is_expired():
            self.context.stop_bot() # Stop the robot, as we are at the correct position
            self.context.transition_to(WaitingAtTurnLeftSignState())

class ApproachingTurnRightSignState(DuckiebotState):
    def __init__(self, tag: AprilTagDetection):
        self._timer = Timer(APPROACHING_SIGN_TIMEOUT_DURATION)
        self._slowdown_timer = Timer(APPROACHING_SIGN_SLOWDOWN_DURATION)
        self._in_slowdown_phase = False
        self._start_slowdown_velocity = math.inf
        self._detection_left_distance = math.inf
        self._detection_right_distance = math.inf
        self._detection_z_distance = tag.transform.translation.z

    def on_enter(self) -> None:
        self._timer.start()
        self.context.publish_FSM_state(LANE_FOLLOWING_FSM_STATE)
        wheel_info = self.context.get_wheel_movement_info()
        self._detection_left_distance = wheel_info.get_left_distance()
        self._detection_right_distance = wheel_info.get_right_distance()

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.WHEEL_MOVEMENT_INFO_RECEIVED:
            if self._in_slowdown_phase:
                vel = ApproachingSignTools.calculate_slowdown_velocity(
                    self._slowdown_timer, self._start_slowdown_velocity)
                self.context.publish_velocity(vel, vel)
                return
            
            if ApproachingSignTools.is_at_slowdown_position(self.context.get_wheel_movement_info(),
                            self._detection_left_distance, self._detection_right_distance,
                                self._detection_z_distance):
                self.context.publish_FSM_state(NORMAL_JOYSTICK_CONTROL_FSM_STATE) # Stop lane following
                self._slowdown_timer.start()
                # Get the average velocity of the wheels at the start of the slowdown phase
                self._start_slowdown_velocity = self.context.get_wheel_movement_info().get_average_velocity()
                self._in_slowdown_phase = True

    def update(self) -> None:
        if self._timer.is_expired():
            rospy.logwarn("Approaching turn right sign timer expired, transitioning to lane following state.")
            self.context.transition_to(LaneFollowingState())
            return

        if self._slowdown_timer.is_expired():
            self.context.stop_bot() # Stop the robot, as we are at the correct position
            self.context.transition_to(WaitingAtTurnRightSignState())

class Autopilot:
    def __init__(self):
        #Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)

        self._sign_tag_id = None
        
        self._state_publisher = rospy.Publisher('/vader/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/vader/obstacle_detector', Int8, self.obstacle_callback, queue_size=1)
        rospy.Subscriber('/vader/autopilot_node/mode', Int8, self.autopilot_control_callback, queue_size=1)

        self.set_lane_following_parameters()

        self._duckiebot = Duckiebot(LaneFollowingState(), self._state_publisher)

        rospy.loginfo("Initialized autopilot node!")

    def autopilot_control_callback(self, msg: Int8):
        if msg.data != 0 and msg.data != 1:
            rospy.logwarn("Unknown autopilot control state received.")
            return
        
        if msg.data == 1:
            self._duckiebot.on_event(DuckieBotEvent.RESUME_COMMAND_RECEIVED)
        else: # msg.data == 0
            self._duckiebot.on_event(DuckieBotEvent.PAUSE_COMMAND_RECEIVED)

    def obstacle_callback(self, msg: Int8):
        if msg.data != 0 and msg.data != 1:
            rospy.logwarn("Unknown obstacle state received.")
            return

        if msg.data == 1:
            self._duckiebot.on_event(DuckieBotEvent.CAR_DETECTED)
        elif msg.data == 0:
            self._duckiebot.on_event(DuckieBotEvent.CAR_REMOVED)

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
        self._duckiebot.stop_bot()  # Stop the robot before shutdown
    
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
