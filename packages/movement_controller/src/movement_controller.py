#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64MultiArray, Int8
from duckietown_msgs.msg import WheelsCmdStamped, FSMState
from abc import ABC, abstractmethod
from enum import Enum

MOVEMENT_CONTROLLER_UPDATE_FREQUENCY = 20.0  # Hz
MOVEMENT_CONTROLLER_UPDATE_PERIOD = 1.0 / MOVEMENT_CONTROLLER_UPDATE_FREQUENCY  # seconds

WHEEL_VELOCITY_STOPPED_THRESHOLD = 0.01  # m/s, threshold to consider the wheel stopped

OVERTAKING_FORWARD_DISTANCE = 0.8  # meters
OVERTAKING_MIDWAY_DISTANCE = OVERTAKING_FORWARD_DISTANCE / 2.0  # meters
AXLE_LENGTH = 0.1
OVERTAKING_WHEEL_OFFSET = 0.09  # meters
OVERTAKING_MANEUVER_DURATION = 6.0  # seconds, duration of the overtaking maneuver
OVERTAKING_TIMEOUT_DURATION = 10.0  # seconds
TURNING_TIMEOUT_DURATION = 10.0  # seconds
STOPPING_TIMEOUT_DURATION = 10.0  # seconds

MAX_VELOCITY = 0.5  # m/s, maximum velocity for the overtaking maneuver
MIN_VELOCITY = 0.1  # m/s, minimum velocity for the overtaking maneuver
OVERTAKING_VELOCITY = 0.3  # m/s

# Generalized logistic function parameters
# These parameters can be tuned based on the desired behavior of the overtaking maneuver
A = 0.0
K = 0.3
B = 40.0
X0 = 0.15
V = 1.0

TRAPEZOIDAL_RULE_N = 100  # Number of intervals for trapezoidal rule integration

OVERTAKING_START_FSM_STATE = 'OVERTAKING_START'
OVERTAKING_SUCCESS_FSM_STATE = 'OVERTAKING_SUCCESS'
OVERTAKING_FAILURE_FSM_STATE = 'OVERTAKING_FAILURE'

TURNING_START_FSM_STATE = 'TURNING_START'
TURNING_SUCCESS_FSM_STATE = 'TURNING_SUCCESS'
TURNING_FAILURE_FSM_STATE = 'TURNING_FAILURE'

STOPPING_START_FSM_STATE = 'STOPPING_START'
STOPPING_SUCCESS_FSM_STATE = 'STOPPING_SUCCESS'
STOPPING_FAILURE_FSM_STATE = 'STOPPING_FAILURE'

class MovementControllerEvent(Enum):
    START_OVERTAKING = 0
    START_TURNING = 1
    START_STOPPING = 2

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
    def get_remaining_time(self) -> float:
        if self._start_time is None:
            return self._duration
        remaining_time = self._duration - (rospy.get_time() - self._start_time)
        return max(0.0, remaining_time)
    def is_timer_less_than_half_expired(self) -> bool:
        if self._start_time is None:
            return False
        elapsed_time = rospy.get_time() - self._start_time
        return elapsed_time < (self._duration / 2.0)
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

    def get_right_info(self):
        return (self._right_distance, self._right_displacement, self._right_velocity)

class MovementController:
    def __init__(self, state: 'MovementControllerState'):
        # Initialize the node
        rospy.init_node('movement_controller_node', anonymous=True)

        self._wheel_movement_info = WheelMovementInfo()

        # Initialize class variables
        self._last_distance_left = 0.0
        self._last_displacement_left = 0.0
        self._last_velocity_left = 0.0
        self._last_distance_right = 0.0
        self._last_displacement_right = 0.0
        self._last_velocity_right = 0.0

        self._state = None

        rospy.Subscriber('/vader/wheel_movement_info', Float64MultiArray, self.wheel_movement_info_callback)
        rospy.Subscriber('/vader/movement_controller_node/goal_overtaking', Int8, self.goal_overtaking_callback)
        rospy.Subscriber('/vader/movement_controller_node/goal_stopping', Int8, self.goal_stopping_callback)
        rospy.Subscriber('/vader/movement_controller_node/goal_turning', Int8, self.goal_turning_callback)
        self._velocity_publisher = rospy.Publisher("/vader/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        self._state_publisher = rospy.Publisher('/vader/fsm_node/mode', FSMState, queue_size=1)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initialized movement_controller node!")
        self.transition_to(state)

    def transition_to(self, state: 'MovementControllerState'):
        self._state = state
        self._state.context = self
        rospy.loginfo(f"Transitioning to state: {type(state).__name__}")
        self._state.on_enter()

    def on_event(self, event: MovementControllerEvent) -> None:
        rospy.loginfo(f"Event received: {event}")
        self._state.on_event(event)

    def update(self):
        self._state.update()

    def publish_fsm_state(self, state: str):
        fsm_state_msg = FSMState()
        fsm_state_msg.header.stamp = rospy.Time.now()
        fsm_state_msg.state = state
        self._state_publisher.publish(fsm_state_msg)

    def wheel_movement_info_callback(self, msg):
        self._wheel_movement_info.update(msg)

    def goal_overtaking_callback(self, msg):
        if msg.data == 1:
            rospy.loginfo("Received goal to overtake")
            self.on_event(MovementControllerEvent.START_OVERTAKING)
        else:
            rospy.logwarn("Invalid overtaking goal received, expected 1 to start overtaking.")

    def goal_stopping_callback(self, msg):
        if msg.data == 1:
            rospy.loginfo("Received goal to stop")
            self.on_event(MovementControllerEvent.START_STOPPING)

    def goal_turning_callback(self, msg):
        if msg.data == 1:
            rospy.loginfo("Received goal to turn left")
            self.on_event(MovementControllerEvent.START_TURNING)
        elif msg.data == 2:
            rospy.loginfo("Received goal to turn right")
            self.on_event(MovementControllerEvent.START_TURNING)
        else:
            rospy.logwarn("Invalid turning goal received, expected 1 for left turn or 2 for right turn.")
    
    def get_wheel_movement_info(self) -> WheelMovementInfo:
        return self._wheel_movement_info
    
    def publish_velocity(self, left_velocity: float, right_velocity: float):
        wheels_cmd = WheelsCmdStamped()
        wheels_cmd.header.stamp = rospy.Time.now()
        wheels_cmd.vel_left = left_velocity
        wheels_cmd.vel_right = right_velocity
        self._velocity_publisher.publish(wheels_cmd)

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
        elif event == MovementControllerEvent.START_TURNING:
            self.context.transition_to(TurningState())
        elif event == MovementControllerEvent.START_STOPPING:
            self.context.transition_to(StoppingState())

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
    def cumulative_track_distance(a: float, b: float, t: float, A: float, K: float, B: float, x0: float,
                                  V: float, midway: float, wheel_offset: float, is_left: bool) -> float:
        return OvertakingTools.trapezoidal_rule(
            OvertakingTools.track_arc_length_integrand, a, b, TRAPEZOIDAL_RULE_N,
            args=(A, K, B, x0, V, midway, wheel_offset, is_left))

class VelocityCalculator:
    @staticmethod
    def calculate_velocity(target_distance: float, current_distance: float) -> float:
        velocity = (target_distance - current_distance) / MOVEMENT_CONTROLLER_UPDATE_PERIOD
        # clamp the velocity to a maximum and minimum value
        return max(MIN_VELOCITY, min(MAX_VELOCITY, velocity))

class OvertakingState(MovementControllerState):
    def __init__(self):
        self._timeout_timer = Timer(OVERTAKING_TIMEOUT_DURATION)
        self._goal_timer = Timer(OVERTAKING_MANEUVER_DURATION)
        self._start_left_distance = 0.0
        self._start_right_distance = 0.0

    def on_enter(self) -> None:
        self.context.publish_fsm_state(OVERTAKING_START_FSM_STATE)
        self._timeout_timer.start()
        self._goal_timer.start()
        wheel_info = self.context.get_wheel_movement_info()
        self._start_left_distance = wheel_info.get_left_info()[0]
        self._start_right_distance = wheel_info.get_right_info()[0]

    def on_event(self, event: MovementControllerEvent) -> None:
        pass

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
        
        wheel_info = self.context.get_wheel_movement_info()
        left_distance = wheel_info.get_left_info()[0]
        right_distance = wheel_info.get_right_info()[0]
        # Calculate the current distances from the starting point
        current_left_distance = left_distance - self._start_left_distance
        current_right_distance = right_distance - self._start_right_distance

        timer_elapsed = self._goal_timer.get_elapsed_time()

        target_left_distance = OvertakingTools.cumulative_track_distance(
            0.0, timer_elapsed, timer_elapsed, A, K, B, X0, V,
                OVERTAKING_MIDWAY_DISTANCE, OVERTAKING_WHEEL_OFFSET, True)
        
        target_right_distance = OvertakingTools.cumulative_track_distance(
            0.0, timer_elapsed, timer_elapsed, A, K, B, X0, V,
                OVERTAKING_MIDWAY_DISTANCE, OVERTAKING_WHEEL_OFFSET, False)
        
        # Calculate the velocities for the left and right wheels
        left_velocity = VelocityCalculator.calculate_velocity(target_left_distance, current_left_distance)
        right_velocity = VelocityCalculator.calculate_velocity(target_right_distance, current_right_distance)

        # Publish the velocities to the wheels
        self.context.publish_velocity(left_velocity, right_velocity)
        rospy.loginfo(f"Overtaking velocities - Left: {left_velocity}, Right: {right_velocity}")

class TurningState(MovementControllerState):
    def __init__(self):
        self._timer = Timer(TURNING_TIMEOUT_DURATION)

    def on_enter(self) -> None:
        self.context.publish_fsm_state(TURNING_START_FSM_STATE)
        self._timer.start()

    def on_event(self, event: MovementControllerEvent) -> None:
        pass

    def update(self) -> None:
        if self._timer.is_expired():
            rospy.logwarn("Turning timed out, transitioning to IdleState")
            self.context.publish_fsm_state(TURNING_FAILURE_FSM_STATE)
            self.context.transition_to(IdleState())
            return

        if 1:
            self.context.publish_fsm_state(TURNING_SUCCESS_FSM_STATE)
            self.context.transition_to(IdleState())

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
