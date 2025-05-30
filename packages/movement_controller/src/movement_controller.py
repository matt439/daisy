#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float64MultiArray, Float64
from duckietown_msgs.msg import WheelsCmdStamped, FSMState
from enum import Enum

AXLE_LENGTH = 0.1  # meters

# Velocity constants
WHEEL_VELOCITY = 0.4  # m/s
WHEEL_TURN_VELOCITY = 0.4  # m/s
MAX_VELOCITY = 0.6  # m/s
MIN_VELOCITY = 0.2  # m/s
WHEEL_VELOCITY_STOPPED_THRESHOLD = 0.01  # m/s, threshold to consider the wheel stopped

# Goal start time period and slowdown scalars
GOAL_START_TIME_PERIOD = 0.5  # seconds
START_TIME_PERIOD_SLOWDOWN_SCALAR = 0.7 # scalar to slow down the wheels during the start time period

# Approach and final slowdown thresholds and scalars
DISTANCE_SLOWDOWN_THRESHOLD_APPROACH = 0.1  # meters
SLOWDOWN_SCALAR_APPROACH = 0.7 # scalar to slow down the wheels when approaching the goal
DISTANCE_SLOWDOWN_THRESHOLD_FINAL = 0.05  # meters
SLOWDOWN_SCALAR_FINAL = 0.5 # scalar to slow down the wheels when near the goal

# Distance completion threshold
DISTANCE_COMPLETE_THRESHOLD = 0.01  # meters

# Number of readings to consider the wheels not moving
ZERO_VELOCITY_READINGS_COUNT_THRESHOLD = 10  # number of readings

# Error correction scalars for left and right wheels
LEFT_ERROR_CORRECTION_SCALAR = 0.88
RIGHT_ERROR_CORRECTION_SCALAR = 1.0 / LEFT_ERROR_CORRECTION_SCALAR

# FSM States
STATE_SUCCESS = 'MOVEMENT_CONTROLLER_SUCCESS'
STATE_FAILURE = 'MOVEMENT_CONTROLLER_FAILURE'
STATE_ACTIVE = 'MOVEMENT_CONTROLLER_ACTIVE'

class VelocityAdjustmentType(Enum):
    STRAIGHT = 0
    TURN = 1

class MovementController:
    def __init__(self):
        # Initialize class variables
        self._last_distance_left = 0.0
        self._last_displacement_left = 0.0
        self._last_velocity_left = 0.0
        self._last_distance_right = 0.0
        self._last_displacement_right = 0.0
        self._last_velocity_right = 0.0

        self._goal_distance_left = 0.0
        self._goal_distance_right = 0.0
        self._dist_goal_active = False
        self._velocity_adjustment_type = VelocityAdjustmentType.STRAIGHT

        self._goal_start_time = time.time()
        self._zero_velocity_readings_count_left = 0
        self._zero_velocity_readings_count_right = 0

        # Initialize the node
        rospy.init_node('movement_controller_node', anonymous=True)

        # Initialise subscribers and publishers
        rospy.Subscriber("/vader/goal_angle", Float64, self.goal_angle_callback)
        rospy.Subscriber("/vader/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber('/vader/wheel_movement_info', Float64MultiArray, self.wheel_movement_info_callback)
        self._velocity_publisher = rospy.Publisher("/vader/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        self._state_publisher = rospy.Publisher('/vader/fsm_node/mode', FSMState, queue_size=1)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initalized movement_controller node!")

    def wheel_movement_info_callback(self, msg):
        self._last_distance_left = msg.data[0]
        self._last_displacement_left = msg.data[1]
        self._last_velocity_left = msg.data[2]
        self._last_distance_right = msg.data[3]
        self._last_displacement_right = msg.data[4]
        self._last_velocity_right = msg.data[5]
        self.handle_goals()

    def rotation_to_distance(self, rotation, axle_length):
        distance = rotation * axle_length / 2.0
        return distance

    def goal_angle_callback(self, msg):
        rospy.loginfo("Received goal angle: %s", msg.data)
        if msg.data == 0.0:
            return
        distance = abs(self.rotation_to_distance(msg.data, AXLE_LENGTH))
        clockwise = True if msg.data < 0.0 else False
        if clockwise:
            left_distance = distance
            right_distance = -distance
        else:
            left_distance = -distance
            right_distance = distance
        self._goal_distance_left = self._last_distance_left + left_distance
        self._goal_distance_right = self._last_distance_right + right_distance
        self._velocity_adjustment_type = VelocityAdjustmentType.TURN
        self._goal_start_time = time.time()
        self._zero_velocity_readings_count_left = 0
        self._zero_velocity_readings_count_right = 0
        self._dist_goal_active = True

    def goal_distance_callback(self, msg):
        rospy.loginfo("Received goal distance: %s", msg.data)
        if msg.data == 0.0:
            return
        self._goal_distance_left = self._last_distance_left + msg.data
        self._goal_distance_right = self._last_distance_right + msg.data
        rospy.loginfo("Last distance left: %s", self._last_distance_left)
        rospy.loginfo("Last distance right: %s", self._last_distance_right)
        rospy.loginfo("Goal distance left: %s", self._goal_distance_left)
        rospy.loginfo("Goal distance right: %s", self._goal_distance_right)
        self._velocity_adjustment_type = VelocityAdjustmentType.STRAIGHT
        self._goal_start_time = time.time()
        self._zero_velocity_readings_count_left = 0
        self._zero_velocity_readings_count_right = 0
        self._dist_goal_active = True
        fsm_msg = FSMState()
        fsm_msg.header.stamp = rospy.Time.now()
        fsm_msg.state = STATE_ACTIVE
        self._state_publisher.publish(fsm_msg)

    def calculate_distance_to_goal(self):
        left = self._goal_distance_left - self._last_distance_left
        right = self._goal_distance_right - self._last_distance_right
        return (left, right)

    def calculate_abs_distance_to_goal(self):
        left, right = self.calculate_distance_to_goal()
        return (abs(left), abs(right))

    def is_distance_goal_complete(self):
        abs_left, abs_right = self.calculate_abs_distance_to_goal()
        if abs_left < DISTANCE_COMPLETE_THRESHOLD or abs_right < DISTANCE_COMPLETE_THRESHOLD:
            return True
        return False
    
    def is_moving_forward(self):
        distance_to_goal = self.calculate_distance_to_goal()
        left_forward = True if distance_to_goal[0] > 0.0 else False
        right_forward = True if distance_to_goal[1] > 0.0 else False
        return (left_forward, right_forward)
    
    def calculate_direction_scalar(self):
        left_forward, right_forward = self.is_moving_forward()
        left_scalar = 1.0 if left_forward else -1.0
        right_scalar = 1.0 if right_forward else -1.0
        return (left_scalar, right_scalar)
    
    def count_zero_velocity_readings(self):
        left_moving, right_moving = self.is_wheels_moving()
        if not left_moving:
            self._zero_velocity_readings_count_left += 1
        if not right_moving:
            self._zero_velocity_readings_count_right += 1
    
    def is_zero_velocity_readings_count_exceeded(self):
        if (self._zero_velocity_readings_count_left > ZERO_VELOCITY_READINGS_COUNT_THRESHOLD or
                    self._zero_velocity_readings_count_right > ZERO_VELOCITY_READINGS_COUNT_THRESHOLD):
            return True
        return False

    def is_wheels_moving(self):
        abs_vel_left, abs_vel_right = self.calculate_abs_velocity()
        return (abs_vel_left > WHEEL_VELOCITY_STOPPED_THRESHOLD, abs_vel_right > WHEEL_VELOCITY_STOPPED_THRESHOLD)
    
    def calculate_abs_velocity(self):
        abs_left = abs(self._last_velocity_left)
        abs_right = abs(self._last_velocity_right)
        return (abs_left, abs_right)
    
    def calculate_maintain_straight_velocity_scalar(self):
        abs_left, abs_right = self.calculate_abs_velocity()

        if abs_left == 0.0 and abs_right == 0.0:
            return (1.0, 1.0)
        else: # adjust the wheel velocities
            # add 0.5 to the scalar to make them closer to 1.0
            left_velocity_scalar = (abs_right / (abs_left + abs_right)) + 0.5
            right_velocity_scalar = (abs_left / (abs_left + abs_right)) + 0.5
        return (left_velocity_scalar, right_velocity_scalar)
    
    def clamp_and_correct_vel_direction(self, left_vel, right_vel):
        # get absolute values of the velocities
        abs_left = abs(left_vel)
        abs_right = abs(right_vel)
        # clamp the velocities to a maximum and minimum value
        left_vel = max(min(abs_left, MAX_VELOCITY), MIN_VELOCITY)
        right_vel = max(min(abs_right, MAX_VELOCITY), MIN_VELOCITY)
        # calculate the direction of the wheels (positive for forward, negative for backward)
        left_direction_scalar, right_direction_scalar = self.calculate_direction_scalar()
        # multiply the velocities by the direction scalars to get the correct direction
        left_vel *= left_direction_scalar
        right_vel *= right_direction_scalar
        return (left_vel, right_vel)
    
    def calculate_start_time_period_slowdown_scalar(self):
        if self.is_goal_start_time_period_complete():
            return (1.0, 1.0)
        return (START_TIME_PERIOD_SLOWDOWN_SCALAR, START_TIME_PERIOD_SLOWDOWN_SCALAR)
    
    def calculate_near_goal_slowdown_scalar(self):
        abs_left, abs_right = self.calculate_abs_distance_to_goal()
        if abs_left < DISTANCE_SLOWDOWN_THRESHOLD_FINAL:
            left_slowdown_scalar = SLOWDOWN_SCALAR_FINAL
        elif abs_left < DISTANCE_SLOWDOWN_THRESHOLD_APPROACH:
            left_slowdown_scalar = SLOWDOWN_SCALAR_APPROACH
        else:
            left_slowdown_scalar = 1.0

        if abs_right < DISTANCE_SLOWDOWN_THRESHOLD_FINAL:
            right_slowdown_scalar = SLOWDOWN_SCALAR_FINAL
        elif abs_right < DISTANCE_SLOWDOWN_THRESHOLD_APPROACH:
            right_slowdown_scalar = SLOWDOWN_SCALAR_APPROACH
        else:
            right_slowdown_scalar = 1.0

        return (left_slowdown_scalar, right_slowdown_scalar)    
    
    def is_goal_start_time_period_complete(self):
        if time.time() - self._goal_start_time > GOAL_START_TIME_PERIOD:
            return True
        return False

    def calculate_adjusted_wheel_velocity(self, velocity_adjustment_type):
        if velocity_adjustment_type == VelocityAdjustmentType.STRAIGHT:
            return self.calculate_straight_adjusted_wheel_velocity()
        elif velocity_adjustment_type == VelocityAdjustmentType.TURN:
            return self.calculate_turn_adjusted_wheel_velocity()
        else:
            rospy.logerr("Invalid velocity adjustment type!")
            return WheelsCmdStamped()
        
    def calculate_turn_adjusted_wheel_velocity(self):
        # calculate direction scalars
        left_direction_scalar, right_direction_scalar = self.calculate_direction_scalar()

        cmd = WheelsCmdStamped()
        cmd.vel_left = WHEEL_TURN_VELOCITY * left_direction_scalar
        cmd.vel_right = WHEEL_TURN_VELOCITY * right_direction_scalar

        # apply error correction scalars
        cmd.vel_left *= LEFT_ERROR_CORRECTION_SCALAR
        cmd.vel_right *= RIGHT_ERROR_CORRECTION_SCALAR
        return cmd
    
    def calculate_straight_adjusted_wheel_velocity(self):
        abs_vel_left, abs_vel_right = self.calculate_abs_velocity()

        # calculate the scaling factors for the wheel velocities
        left_maintain_velocity_scalar, right_maintain_velocity_scalar = self.calculate_maintain_straight_velocity_scalar()
        
        cmd = WheelsCmdStamped()
        if abs_vel_left > abs_vel_right: # left wheel is faster
            # slow down the left wheel
            cmd.vel_left = WHEEL_VELOCITY * left_maintain_velocity_scalar
            cmd.vel_right = WHEEL_VELOCITY * right_maintain_velocity_scalar
        elif abs_vel_right > abs_vel_left: # right wheel is faster
            # slow down the right wheel
            cmd.vel_left = WHEEL_VELOCITY * left_maintain_velocity_scalar
            cmd.vel_right = WHEEL_VELOCITY * right_maintain_velocity_scalar
        else: # both wheels are moving at the same speed
            # maintain the same speed
            cmd.vel_left = WHEEL_VELOCITY
            cmd.vel_right = WHEEL_VELOCITY

        # apply the start time period slowdown scalars (if in the start time period)
        left_start_time_period_slowdown_scalar, right_start_time_period_slowdown_scalar = self.calculate_start_time_period_slowdown_scalar()
        cmd.vel_left *= left_start_time_period_slowdown_scalar
        cmd.vel_right *= right_start_time_period_slowdown_scalar

        # slow down the wheels if they are too close to the goal to avoid overshooting
        if self.is_goal_start_time_period_complete(): # only apply this if the start time period is complete
            left_slowdown_scalar, right_slowdown_scalar = self.calculate_near_goal_slowdown_scalar()
            cmd.vel_left *= left_slowdown_scalar
            cmd.vel_right *= right_slowdown_scalar

        # apply error correction scalars
        cmd.vel_left *= LEFT_ERROR_CORRECTION_SCALAR
        cmd.vel_right *= RIGHT_ERROR_CORRECTION_SCALAR

        # clamp the velocities to a maximum and minimum value and correct the direction
        cmd.vel_left, cmd.vel_right = self.clamp_and_correct_vel_direction(cmd.vel_left, cmd.vel_right)
        return cmd

    def handle_distance_goal(self):
        cmd = WheelsCmdStamped()
        cmd.vel_left = 0.0
        cmd.vel_right = 0.0

        if self.is_distance_goal_complete():
            self._dist_goal_active = False
            rospy.loginfo("Distance goal complete!")
            rospy.loginfo("Left wheel final displacement: %s", self._last_distance_left - self._goal_distance_left)
            rospy.loginfo("Right wheel final displacement: %s", self._last_distance_right - self._goal_distance_right)
            fsm_msg = FSMState()
            fsm_msg.header.stamp = rospy.Time.now()
            fsm_msg.state = STATE_SUCCESS
            self._state_publisher.publish(fsm_msg)
        elif self.is_zero_velocity_readings_count_exceeded() and self.is_goal_start_time_period_complete():
            # one or both wheels are not moving and the goal start time period is complete
            rospy.logerr("One or both wheels are not moving in handle_distance_goal()!")
            rospy.logerr("This should not happen!")
            rospy.logerr("Left wheel final displacement: %s", self._last_distance_left - self._goal_distance_left)
            rospy.logerr("Right wheel final displacement: %s", self._last_distance_right - self._goal_distance_right)
            # print zero velocity readings count
            rospy.logerr("Left wheel zero velocity readings count: %s", self._zero_velocity_readings_count_left)
            rospy.logerr("Right wheel zero velocity readings count: %s", self._zero_velocity_readings_count_right)
            fsm_msg = FSMState()
            fsm_msg.header.stamp = rospy.Time.now()
            fsm_msg.state = STATE_FAILURE
            self._state_publisher.publish(fsm_msg)
            self.reset()
        else: # both wheels are moving
            self.count_zero_velocity_readings()
            cmd = self.calculate_adjusted_wheel_velocity(self._velocity_adjustment_type)
            
        self._velocity_publisher.publish(cmd)
    
    def handle_goals(self):
        if self._dist_goal_active:
            self.handle_distance_goal()

    def reset(self):
        self._goal_distance_left = 0.0
        self._goal_distance_right = 0.0
        self._dist_goal_active = False
        self._velocity_adjustment_type = VelocityAdjustmentType.STRAIGHT

        self._goal_start_time = time.time()
        self._zero_velocity_readings_count_left = 0
        self._zero_velocity_readings_count_right = 0

if __name__ == '__main__':
    try:
        movement_controller = MovementController()
        # no need to call run() as it relies on the callbacks from wheel_movement_info to update the state
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

