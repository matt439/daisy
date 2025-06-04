#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64MultiArray, Int8
from duckietown_msgs.msg import WheelsCmdStamped, FSMState, Twist2DStamped, AprilTagDetectionArray
from abc import ABC, abstractmethod
from enum import Enum

MOVEMENT_CONTROLLER_UPDATE_FREQUENCY = 20.0  # Hz

MAX_VELOCITY = 0.8  # m/s, maximum velocity for the overtaking maneuver
MIN_VELOCITY = 0.0  # m/s, minimum velocity for the overtaking maneuver

# Overtaking parameters
OVERTAKING_TIMEOUT_DURATION = 15.0  # seconds
OVERTAKING_MANEUVER_DURATION = 5.0  # seconds, duration of the overtaking maneuver
OVERTAKING_FORWARD_DISTANCE = 0.5  # meters
OVERTAKING_MIDWAY_DISTANCE = 0.25 # meters, where the piecewise function is split into two parts 
# AXLE_LENGTH = 0.1
OVERTAKING_WHEEL_OFFSET = 0.09  # meters
# Generalized logistic function parameters for overtaking
A = 0.0
K = 0.25
B = 70.0
X0 = 0.08
V = 1.0
TRAPEZOIDAL_RULE_N = 100  # Number of intervals for trapezoidal rule integration
OVERTAKING_START_FSM_STATE = 'OVERTAKING_START'
OVERTAKING_SUCCESS_FSM_STATE = 'OVERTAKING_SUCCESS'
OVERTAKING_FAILURE_FSM_STATE = 'OVERTAKING_FAILURE'

# Turning parameters
TURNING_TIMEOUT_DURATION = 10.0  # seconds
TURN_MAX_VELOCITY_ADJUSTMENT_SCALAR = 1.5
TURN_MIN_VELOCITY_ADJUSTMENT_SCALAR = 0.75
AXLE_LENGTH = 0.1  # meters, distance between the two wheels

TURN_LEFT_RIGHT_WHEEL_RADIUS = 0.39  # meters, radius of the right wheel during left turn
TURN_LEFT_LEFT_WHEEL_RADIUS = TURN_LEFT_RIGHT_WHEEL_RADIUS - AXLE_LENGTH  # meters, radius of the left wheel during left turn
TURN_LEFT_LEFT_WHEEL_DISTANCE = math.pi * TURN_LEFT_LEFT_WHEEL_RADIUS / 2.0  # meters, distance traveled by the left wheel during left turn
TURN_LEFT_RIGHT_WHEEL_DISTANCE = math.pi * TURN_LEFT_RIGHT_WHEEL_RADIUS / 2.0  # meters, distance traveled by the right wheel during left turn
TURN_LEFT_MANEUVER_DURATION = 2.0  # seconds
TURN_LEFT_VELOCITY_LEFT = TURN_LEFT_LEFT_WHEEL_DISTANCE / TURN_LEFT_MANEUVER_DURATION  # m/s for left wheel
TURN_LEFT_VELOCITY_LEFT_MAX = TURN_LEFT_VELOCITY_LEFT * TURN_MAX_VELOCITY_ADJUSTMENT_SCALAR  # m/s for left wheel max
TURN_LEFT_VELOCITY_LEFT_MIN = TURN_LEFT_VELOCITY_LEFT * TURN_MIN_VELOCITY_ADJUSTMENT_SCALAR  # m/s for left wheel min
TURN_LEFT_VELOCITY_RIGHT = TURN_LEFT_RIGHT_WHEEL_DISTANCE / TURN_LEFT_MANEUVER_DURATION  # m/s for right wheel
TURN_LEFT_VELOCITY_RIGHT_MAX = TURN_LEFT_VELOCITY_RIGHT * TURN_MAX_VELOCITY_ADJUSTMENT_SCALAR  # m/s for right wheel max
TURN_LEFT_VELOCITY_RIGHT_MIN = TURN_LEFT_VELOCITY_RIGHT * TURN_MIN_VELOCITY_ADJUSTMENT_SCALAR  # m/s for right wheel min

TURN_RIGHT_LEFT_WHEEL_RADIUS = 0.17  # meters, radius of the left wheel during right turn
TURN_RIGHT_RIGHT_WHEEL_RADIUS = TURN_RIGHT_LEFT_WHEEL_RADIUS - AXLE_LENGTH  # meters, radius of the right wheel during right turn
TURN_RIGHT_LEFT_WHEEL_DISTANCE = math.pi * TURN_RIGHT_LEFT_WHEEL_RADIUS / 2.0  # meters, distance traveled by the left wheel during right turn
TURN_RIGHT_RIGHT_WHEEL_DISTANCE = math.pi * TURN_RIGHT_RIGHT_WHEEL_RADIUS / 2.0  # meters, distance traveled by the right wheel during right turn
TURN_RIGHT_MANEUVER_DURATION = 1.0  # seconds
TURN_RIGHT_VELOCITY_LEFT = TURN_RIGHT_LEFT_WHEEL_DISTANCE / TURN_RIGHT_MANEUVER_DURATION  # m/s for left wheel
TURN_RIGHT_VELOCITY_RIGHT = TURN_RIGHT_RIGHT_WHEEL_DISTANCE / TURN_RIGHT_MANEUVER_DURATION  # m/s for right wheel
TURN_RIGHT_VELOCITY_LEFT_MAX = TURN_RIGHT_VELOCITY_LEFT * TURN_MAX_VELOCITY_ADJUSTMENT_SCALAR  # m/s for left wheel max
TURN_RIGHT_VELOCITY_LEFT_MIN = TURN_RIGHT_VELOCITY_LEFT * TURN_MIN_VELOCITY_ADJUSTMENT_SCALAR  # m/s for left wheel min
TURN_RIGHT_VELOCITY_RIGHT_MAX = TURN_RIGHT_VELOCITY_RIGHT * TURN_MAX_VELOCITY_ADJUSTMENT_SCALAR  # m/s for right wheel max
TURN_RIGHT_VELOCITY_RIGHT_MIN = TURN_RIGHT_VELOCITY_RIGHT * TURN_MIN_VELOCITY_ADJUSTMENT_SCALAR  # m/s for right wheel min
TURNING_START_FSM_STATE = 'TURNING_START'
TURNING_SUCCESS_FSM_STATE = 'TURNING_SUCCESS'
TURNING_FAILURE_FSM_STATE = 'TURNING_FAILURE'

# Stopping parameters
STOPPING_TIMEOUT_DURATION = 10.0  # seconds
WHEEL_VELOCITY_STOPPED_THRESHOLD = 0.01  # m/s, threshold to consider the wheel stopped
STOPPING_START_FSM_STATE = 'STOPPING_START'
STOPPING_SUCCESS_FSM_STATE = 'STOPPING_SUCCESS'
STOPPING_FAILURE_FSM_STATE = 'STOPPING_FAILURE'

# Approaching sign parameters
APPROACHING_SIGN_TIMEOUT_DURATION = 10.0  # seconds

FOLLOW_ANGULAR_VELOCITY = 0.1 # rad/s
FOLLOW_ANGULAR_VELOCITY_MAX = 0.2 # rad/s
FOLLOW_ANGULAR_VELOCITY_MIN = 0.0 # rad/s
# FOLLOW_ANGULAR_VELOCITY_AVG_DISTANCE = 0.3
FOLLOW_X_DISTANCE_TARGET = 0.25 # meter, the sign should be to the right of the bot
FOLLOW_X_DISTANCE_THRESHOLD = 0.03 # meter

FOLLOW_Z_DISTANCE_TARGET = 0.6 # meter, sign is this distance in front of the stop line
FOLLOW_Z_DISTANCE_THRESHOLD = 0.05 # meter
FOLLOW_LINEAR_VELOCITY = 0.2 # m/s
FOLLOW_LINEAR_VELOCITY_MAX = 0.3 # m/s
FOLLOW_LINEAR_VELOCITY_MIN = 0.1 # m/s

APPROACHING_SIGN_START_FSM_STATE = 'APPROACHING_SIGN_START'
APPROACHING_SIGN_SUCCESS_FSM_STATE = 'APPROACHING_SIGN_SUCCESS'
APPROACHING_SIGN_FAILURE_FSM_STATE = 'APPROACHING_SIGN_FAILURE'

class MovementControllerEvent(Enum):
    START_OVERTAKING = 0
    START_APPROACHING_SIGN = 1
    START_TURNING_LEFT = 2
    START_TURNING_RIGHT = 3
    START_STOPPING = 4
    WHEEL_MOVEMENT_INFO_UPDATED = 5
    APRIL_TAG_DETECTION_UPDATED = 6

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

    def get_left_info(self):
        return (self._left_distance, self._left_displacement, self._left_velocity)
    def get_left_distance(self):
        return self._left_distance
    def get_left_displacement(self):
        return self._left_displacement
    def get_left_velocity(self):
        return self._left_velocity
    def get_right_info(self):
        return (self._right_distance, self._right_displacement, self._right_velocity)
    def get_right_distance(self):
        return self._right_distance
    def get_right_displacement(self):
        return self._right_displacement
    def get_right_velocity(self):
        return self._right_velocity
    
class AprilTagDetection:
    def __init__(self):
        self._tag_id = 0.0
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0

    def update(self, tag_id: int, x: float, y: float, z: float):
        self._tag_id = tag_id
        self._x = x
        self._y = y
        self._z = z

    def get_tag_id(self) -> int:
        return self._tag_id
    def get_x(self) -> float:
        return self._x
    def get_y(self) -> float:
        return self._y
    def get_z(self) -> float:
        return self._z

class MovementController:
    def __init__(self, state: 'MovementControllerState'):
        # Initialize the node
        rospy.init_node('movement_controller_node', anonymous=True)

        self._wheel_movement_info = WheelMovementInfo()
        self._april_tag_detections_map = {}  # Map to hold tag_id to AprilTagDetection object

        # Initialize class variables
        self._last_distance_left = 0.0
        self._last_displacement_left = 0.0
        self._last_velocity_left = 0.0
        self._last_distance_right = 0.0
        self._last_displacement_right = 0.0
        self._last_velocity_right = 0.0

        self._sign_tag_id = None

        self._state = None

        rospy.Subscriber('/vader/wheel_movement_info', Float64MultiArray, self.wheel_movement_info_callback)
        rospy.Subscriber('/vader/movement_controller_node/goal_overtaking', Int8, self.goal_overtaking_callback)
        rospy.Subscriber('/vader/movement_controller_node/goal_stopping', Int8, self.goal_stopping_callback)
        rospy.Subscriber('/vader/movement_controller_node/goal_turning', Int8, self.goal_turning_callback)
        rospy.Subscriber('/vader/movement_controller_node/goal_approaching_sign', Int8, self.goal_approaching_sign_callback)
        rospy.Subscriber('/vader/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback)
        self._velocity_publisher = rospy.Publisher("/vader/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        self._state_publisher = rospy.Publisher('/vader/fsm_node/mode', FSMState, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/vader/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initialized movement_controller node!")
        self.transition_to(state)

    def transition_to(self, state: 'MovementControllerState'):
        self._state = state
        self._state._context = self
        rospy.loginfo(f"MovementController transitioning to state: {type(state).__name__}")
        self._state.on_enter()

    def on_event(self, event: MovementControllerEvent) -> None:
        if event is not MovementControllerEvent.APRIL_TAG_DETECTION_UPDATED:
            # Log all events except APRIL_TAG_DETECTION_UPDATED
            # as it clogs the log with too many messages
            rospy.loginfo(f"MovementController event received: {event}")
        self._state.on_event(event)

    def update(self):
        self._state.update()

    def publish_fsm_state(self, state: str):
        fsm_state_msg = FSMState()
        fsm_state_msg.header.stamp = rospy.Time.now()
        fsm_state_msg.state = state
        self._state_publisher.publish(fsm_state_msg)

    def publish_cmd_vel(self, twist: Twist2DStamped):
        self.cmd_vel_pub.publish(twist)

    def wheel_movement_info_callback(self, msg):
        self._wheel_movement_info.update(msg)
        if self._state is not None:
            self._state.on_event(MovementControllerEvent.WHEEL_MOVEMENT_INFO_UPDATED)

    def tag_callback(self, msg):        
        for detection in msg.detections:
            id = detection.tag_id
            x = detection.transform.translation.x
            y = detection.transform.translation.y
            z = detection.transform.translation.z
            
            if id not in self._april_tag_detections_map:
                self._april_tag_detections_map[id] = AprilTagDetection()
            
            self._april_tag_detections_map[id].update(id, x, y, z)
        
            if self._state is not None:
                self._state.on_event(MovementControllerEvent.APRIL_TAG_DETECTION_UPDATED)

    def goal_approaching_sign_callback(self, msg):
        self._sign_tag_id = msg.data
        self.on_event(MovementControllerEvent.START_APPROACHING_SIGN)

    def goal_overtaking_callback(self, msg):
        if msg.data == 1:
            rospy.loginfo("MovementController received goal to overtake")
            self.on_event(MovementControllerEvent.START_OVERTAKING)
        else:
            rospy.logwarn("Invalid overtaking goal received, expected 1 to start overtaking.")

    def goal_stopping_callback(self, msg):
        if msg.data == 1:
            rospy.loginfo("MovementController received goal to stop")
            self.on_event(MovementControllerEvent.START_STOPPING)

    def goal_turning_callback(self, msg):
        if msg.data == 1:
            rospy.loginfo("MovementController received goal to turn left")
            self.on_event(MovementControllerEvent.START_TURNING_LEFT)
        elif msg.data == 2:
            rospy.loginfo("MovementController received goal to turn right")
            self.on_event(MovementControllerEvent.START_TURNING_RIGHT)
        else:
            rospy.logwarn("Invalid turning goal received, expected 1 for left turn or 2 for right turn.")
    
    def get_wheel_movement_info(self) -> WheelMovementInfo:
        return self._wheel_movement_info
    
    def get_sign_tag_id(self) -> int:
        return self._sign_tag_id
    
    def publish_velocity(self, left_velocity: float, right_velocity: float):
        wheels_cmd = WheelsCmdStamped()
        wheels_cmd.header.stamp = rospy.Time.now()
        wheels_cmd.vel_left = left_velocity
        wheels_cmd.vel_right = right_velocity
        self._velocity_publisher.publish(wheels_cmd)

    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.publish_cmd_vel(cmd_msg)

    def run(self):
        rate = rospy.Rate(MOVEMENT_CONTROLLER_UPDATE_FREQUENCY)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

class MovementControllerState(ABC):
    @property
    def context(self) -> MovementController:
        return self._context

    @context.setter
    def context(self, context: MovementController) -> None:
        self._context = context

    @abstractmethod
    def on_enter(self) -> None:
        pass

    @abstractmethod
    def on_event(self, event: MovementControllerEvent) -> None:
        pass

    @abstractmethod
    def update(self) -> None:
        pass

class IdleState(MovementControllerState):
    def __init__(self):
        pass

    def on_enter(self) -> None:
        pass

    def on_event(self, event: MovementControllerEvent) -> None:
        if event == MovementControllerEvent.START_OVERTAKING:
            self.context.transition_to(OvertakingState())
        elif event == MovementControllerEvent.START_TURNING_LEFT:
            self.context.transition_to(TurningState(is_left_turn=True))
        elif event == MovementControllerEvent.START_TURNING_RIGHT:
            self.context.transition_to(TurningState(is_left_turn=False))
        elif event == MovementControllerEvent.START_STOPPING:
            self.context.transition_to(StoppingState())
        elif event == MovementControllerEvent.START_APPROACHING_SIGN:
            self.context.transition_to(ApproachingSignState(self.context.get_sign_tag_id()))

    def update(self) -> None:
        pass

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
    def calculate_velocity(target_distance: float, current_distance: float, time: float) -> float:
        if target_distance <= current_distance:
            velocity = 0.0  # No need to move if we are already at or beyond the target distance
        else: # Calculate the velocity needed to reach the target distance in the given time
            velocity = (target_distance - current_distance) / time

        # Clamp the velocity to the maximum and minimum values
        return max(min(velocity, MAX_VELOCITY), MIN_VELOCITY)

class OvertakingState(MovementControllerState):
    def __init__(self):
        self._timeout_timer = Timer(OVERTAKING_TIMEOUT_DURATION)
        self._goal_timer = Timer(OVERTAKING_MANEUVER_DURATION)
        self._start_left_distance = 0.0
        self._start_right_distance = 0.0
        self._last_wheel_info_time = None

    def on_enter(self) -> None:
        self.context.publish_fsm_state(OVERTAKING_START_FSM_STATE)
        self._timeout_timer.start()
        self._goal_timer.start()
        wheel_info = self.context.get_wheel_movement_info()
        self._start_left_distance = wheel_info.get_left_distance()
        self._start_right_distance = wheel_info.get_right_distance()
        self._last_wheel_info_time = rospy.get_time()

    def on_event(self, event: MovementControllerEvent) -> None:
        if event == MovementControllerEvent.WHEEL_MOVEMENT_INFO_UPDATED:
            if self._last_wheel_info_time is not None: # If on_enter has been called
                self.control_bot()

    def update(self) -> None:
        if self._timeout_timer.is_expired():
            rospy.logwarn("Overtaking timed out, transitioning to IdleState")
            self.context.publish_fsm_state(OVERTAKING_FAILURE_FSM_STATE)
            self.context.transition_to(IdleState())
            return

        if self._goal_timer.is_expired():
            self.context.publish_fsm_state(OVERTAKING_SUCCESS_FSM_STATE)
            self.context.transition_to(IdleState())
            return
        
    def control_bot(self):
        wheel_info = self.context.get_wheel_movement_info()
        left_distance = wheel_info.get_left_distance()
        right_distance = wheel_info.get_right_distance()

        # Calculate the current distances from the starting point
        current_left_distance = left_distance - self._start_left_distance
        current_right_distance = right_distance - self._start_right_distance

        timer_elapsed = self._goal_timer.get_elapsed_time()

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
            target_left_distance, current_left_distance, elapsed_time)
        right_velocity = VelocityCalculator.calculate_velocity(
            target_right_distance, current_right_distance, elapsed_time)

        # Publish the velocities to the wheels
        self.context.publish_velocity(left_velocity, right_velocity)
        rospy.loginfo(f"Vel: L: {left_velocity:.2f} m/s, R: {right_velocity:.2f} m/s | "
                    f"Curr Dist: L: {current_left_distance:.2f} m, R: {current_right_distance:.2f} m | "
                        f"Target Dist: L: {target_left_distance:.2f} m, R: {target_right_distance:.2f} m | "
                            f"Time: {timer_elapsed:.2f} s | ")

class ApproachingSignTools:
    @staticmethod
    def calculate_abs_proportional_follow_angular_velocity(x):
        # Scale the angular velocity based on the distance from the target x position
        vel = FOLLOW_ANGULAR_VELOCITY * abs(x) / FOLLOW_X_DISTANCE_THRESHOLD
        # Clamp the velocity to a maximum value
        if vel > FOLLOW_ANGULAR_VELOCITY_MAX:
            vel = FOLLOW_ANGULAR_VELOCITY_MAX
        elif vel < FOLLOW_ANGULAR_VELOCITY_MIN:
            vel = FOLLOW_ANGULAR_VELOCITY_MIN
        return vel

    @staticmethod
    def calculate_follow_angular_velocity(x):
        # If the object is too close, stop moving
        if abs(x) < FOLLOW_X_DISTANCE_THRESHOLD:
            rospy.loginfo("Object is within angular target threshold distance.")
            return 0.0
        
        vel = ApproachingSignTools.calculate_abs_proportional_follow_angular_velocity(x)
        if x > 0.0:
            vel = -vel
        return vel
    
    @staticmethod
    def calculate_abs_proportional_follow_linear_velocity(z):
        # If the object is closer than the target distance, decrease the velocity
        # If the object is further than the target distance, increase the velocity
        # The velocity is proportional to the distance
        vel = FOLLOW_LINEAR_VELOCITY * abs(z) / FOLLOW_Z_DISTANCE_TARGET
        # Clamp the velocity to a maximum value
        if vel > FOLLOW_LINEAR_VELOCITY_MAX:
            vel = FOLLOW_LINEAR_VELOCITY_MAX
        elif vel < FOLLOW_LINEAR_VELOCITY_MIN:
            vel = FOLLOW_LINEAR_VELOCITY_MIN
        return vel

    @staticmethod
    def calculate_follow_linear_velocity(z):
        # check if the object is within the threshold distance
        if abs(z - FOLLOW_Z_DISTANCE_TARGET) < FOLLOW_Z_DISTANCE_THRESHOLD:
            rospy.loginfo("Object is within linear threshold distance.")
            return 0.0
        vel = ApproachingSignTools.calculate_abs_proportional_follow_linear_velocity(z)
        if z < FOLLOW_Z_DISTANCE_TARGET:
            vel = -vel
        return vel

    @staticmethod
    def follow_object(x, z) -> Twist2DStamped:
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = ApproachingSignTools.calculate_follow_linear_velocity(z)
        cmd_msg.omega = ApproachingSignTools.calculate_follow_angular_velocity(x)
        return cmd_msg

class ApproachingSignState(MovementControllerState):
    def __init__(self, tag_id: int):
        self._tag_id = tag_id
        self._timer = Timer(APPROACHING_SIGN_TIMEOUT_DURATION)

    def on_enter(self) -> None:
        self.context.publish_fsm_state(APPROACHING_SIGN_START_FSM_STATE)
        self._timer.start()

    def on_event(self, event: MovementControllerEvent) -> None:
        if event == MovementControllerEvent.APRIL_TAG_DETECTION_UPDATED:
            self.control_bot()

    def update(self) -> None:
        if self._timer.is_expired():
            rospy.logwarn("Approaching sign timed out, transitioning to IdleState")
            self.context.publish_fsm_state(APPROACHING_SIGN_FAILURE_FSM_STATE)
            self.context.stop_robot()
            self.context.transition_to(IdleState())
            return

        if self.is_at_sign():
            self.context.publish_fsm_state(APPROACHING_SIGN_SUCCESS_FSM_STATE)
            self.context.stop_robot()
            self.context.transition_to(IdleState())

    def control_bot(self):
        tag_detection = self.context._april_tag_detections_map.get(self._tag_id)
        if tag_detection:
            x = tag_detection.get_x()
            z = tag_detection.get_z()
            rospy.loginfo(f"Tag ID {self._tag_id} detected at x: {x}, z: {z}")
            self.context.publish_cmd_vel(
                ApproachingSignTools.follow_object(x - FOLLOW_X_DISTANCE_TARGET, z))
        else:
            rospy.logwarn(f"Tag ID {self._tag_id} not found in detections.")

    def is_at_sign(self) -> bool:
        tag_detection = self.context._april_tag_detections_map.get(self._tag_id)
        if tag_detection:
            z = tag_detection.get_z()
            return abs(z - FOLLOW_Z_DISTANCE_TARGET) < FOLLOW_Z_DISTANCE_THRESHOLD
        else:
            rospy.logwarn(f"Tag ID {self._tag_id} not found in detections.")
            return False

class TurningTools:
    @staticmethod
    def calculate_turning_velocity(is_left_turn: bool,
                                   current_vel: float, target_vel: float) -> float:
        if target_vel == 0.0:
            rospy.logwarn("Target velocity is zero, cannot adjust current velocity.")
            return 0.0
        
        diff = target_vel - current_vel
        new_vel = target_vel + diff

        if is_left_turn:
            if new_vel > TURN_LEFT_VELOCITY_LEFT_MAX:
                return TURN_LEFT_VELOCITY_LEFT_MAX
            elif new_vel < TURN_LEFT_VELOCITY_LEFT_MIN:
                return TURN_LEFT_VELOCITY_LEFT_MIN
        else:
            if new_vel > TURN_RIGHT_VELOCITY_LEFT_MAX:
                return TURN_RIGHT_VELOCITY_LEFT_MAX
            elif new_vel < TURN_RIGHT_VELOCITY_LEFT_MIN:
                return TURN_RIGHT_VELOCITY_LEFT_MIN
        return new_vel

class TurningState(MovementControllerState):
    def __init__(self, is_left_turn: bool):
        self._timeout_timer = Timer(TURNING_TIMEOUT_DURATION)
        self._is_left_turn = is_left_turn
        self._goal_timer = Timer(TURN_LEFT_MANEUVER_DURATION if is_left_turn \
                                                else TURN_RIGHT_MANEUVER_DURATION)

    def on_enter(self) -> None:
        self.context.publish_fsm_state(TURNING_START_FSM_STATE)
        self._timeout_timer.start()
        self._goal_timer.start()

    def on_event(self, event: MovementControllerEvent) -> None:
        if event == MovementControllerEvent.WHEEL_MOVEMENT_INFO_UPDATED:
            self.control_bot()

    def update(self) -> None:
        if self._timeout_timer.is_expired():
            rospy.logwarn("Turning timed out, transitioning to IdleState")
            self.context.publish_fsm_state(TURNING_FAILURE_FSM_STATE)
            self.context.transition_to(IdleState())
            return

        if self._goal_timer.is_expired():
            self.context.publish_fsm_state(TURNING_SUCCESS_FSM_STATE)
            self.context.transition_to(IdleState())

    def control_bot(self):
        wheel_info = self.context.get_wheel_movement_info()
        measured_left_velocity = wheel_info.get_left_velocity()
        measured_right_velocity = wheel_info.get_right_velocity()

        if self._is_left_turn:
            left_velocity = TurningTools.calculate_turning_velocity(
                True, measured_left_velocity, TURN_LEFT_VELOCITY_LEFT)
            right_velocity = TurningTools.calculate_turning_velocity(
                True, measured_right_velocity, TURN_LEFT_VELOCITY_RIGHT)
        else:
            left_velocity = TurningTools.calculate_turning_velocity(
                False, measured_left_velocity, TURN_RIGHT_VELOCITY_LEFT)
            right_velocity = TurningTools.calculate_turning_velocity(
                False, measured_right_velocity, TURN_RIGHT_VELOCITY_RIGHT)
            
        self.context.publish_velocity(left_velocity, right_velocity)
        # Log the velocities for debugging
        rospy.loginfo(f"Turning Vel - L: {left_velocity:.2f} m/s, R: {right_velocity:.2f} m/s | "
                      f"Measured Vel - L: {measured_left_velocity:.2f} m/s, R: {measured_right_velocity:.2f} m/s | "
                      f"Is Left Turn: {self._is_left_turn} | "
                      f"Goal Timer: {self._goal_timer.get_elapsed_time():.2f} s | "
                      f"Target vel L turn: L: {TURN_LEFT_VELOCITY_LEFT:.2f} m/s, R: {TURN_LEFT_VELOCITY_RIGHT:.2f} m/s")
        

class StoppingState(MovementControllerState):
    def __init__(self):
        self._timer = Timer(STOPPING_TIMEOUT_DURATION)

    def on_enter(self) -> None:
        self.context.publish_fsm_state(STOPPING_START_FSM_STATE)
        self._timer.start()
        self.send_stop_command()

    def on_event(self, event: MovementControllerEvent) -> None:
        pass

    def update(self) -> None:
        if self._timer.is_expired():
            rospy.logwarn("Stopping timed out, transitioning to IdleState")
            self.context.publish_fsm_state(STOPPING_FAILURE_FSM_STATE)
            self.context.transition_to(IdleState())
            return

        if self.is_wheels_stopped():
            self.context.publish_fsm_state(STOPPING_SUCCESS_FSM_STATE)
            self.context.transition_to(IdleState())
            return

        self.send_stop_command()

    def send_stop_command(self):
        self.context.publish_velocity(0.0, 0.0)

    def is_wheels_stopped(self) -> bool:
        wheel_info = self.context.get_wheel_movement_info()
        left_velocity = wheel_info.get_left_info()[2]
        right_velocity = wheel_info.get_right_info()[2]
        return abs(left_velocity) < WHEEL_VELOCITY_STOPPED_THRESHOLD and \
               abs(right_velocity) < WHEEL_VELOCITY_STOPPED_THRESHOLD

if __name__ == '__main__':
    try:
        movement_controller = MovementController(IdleState())
        movement_controller.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
