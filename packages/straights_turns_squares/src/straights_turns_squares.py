#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64MultiArray, Float64
from duckietown_msgs.msg import WheelsCmdStamped

AXLE_LENGTH = 0.1  # meters
WHEEL_VELOCITY = 0.5  # m/s
DISTANCE_COMPLETE_THRESHOLD = 0.01  # meters
DISTANCE_SLOWDOWN_THRESHOLD_FINAL = 0.05  # meters
SLOWDOWN_FACTOR_FINAL = 0.2
DISTANCE_SLOWDOWN_THRESHOLD_APPROACH = 0.2  # meters
SLOWDOWN_FACTOR_APPROACH = 0.5
MAX_VELOCITY = 0.7  # m/s
MIN_VELOCITY = 0.3  # m/s

class StraightsTurnsSquares:
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

        self._square_goal_active = False
        self._square_edges_completed = 0
        self._square_edge_length = 0.0
        self._square_straight_started = False
        self._square_straight_complete = False
        self._square_turn_started = False
        self._square_turn_complete = False

        # Initialize the node
        rospy.init_node('straights_turns_squares_node', anonymous=True)

        # Initialise subscribers and publishers
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/square_edge_length", Float64, self.square_callback)
        rospy.Subscriber('/wheel_movement_info', Float64MultiArray, self.wheel_movement_info_callback)
        self._velocity_publisher = rospy.Publisher("/vader/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        self._goal_angle_publisher = rospy.Publisher('/goal_angle', Float64, queue_size=1)
        self._goal_distance_publisher = rospy.Publisher('/goal_distance', Float64, queue_size=1)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initalized node!")

    def wheel_movement_info_callback(self, msg):
        self._last_distance_left = msg.data[0]
        self._last_displacement_left = msg.data[1]
        self._last_velocity_left = msg.data[2]
        self._last_distance_right = msg.data[3]
        self._last_displacement_right = msg.data[4]
        self._last_velocity_right = msg.data[5]
        #self._new_wheel_movement_info = True
        self.handle_goals()

    def rotation_to_distance(self, rotation, axle_length):
        distance = rotation * axle_length / 2.0
        return distance

    def goal_angle_callback(self, msg):
        rospy.loginfo("Received goal angle: %s", msg.data)
        if msg.data == 0.0:
            return
        distance = self.rotation_to_distance(msg.data, AXLE_LENGTH)
        clockwise = True if msg.data < 0.0 else False
        if clockwise:
            left_distance = distance
            right_distance = -distance
        else:
            left_distance = -distance
            right_distance = distance
        self._goal_distance_left = self._last_distance_left + left_distance
        self._goal_distance_right = self._last_distance_right + right_distance
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
        self._dist_goal_active = True

    def square_callback(self, msg):
        rospy.loginfo("Received square edge length: %s", msg.data)
        self._square_edges_completed = 0
        self._square_straight_started = False
        self._square_straight_complete = False
        self._square_turn_started = False
        self._square_turn_complete = False
        self._square_edge_length = msg.data
        self._square_goal_active = True

    def calculate_distance_to_goal(self):
        left = self._goal_distance_left - self._last_distance_left
        right = self._goal_distance_right - self._last_distance_right
        return (left, right)

    def calculate_abs_distance_to_goal(self):
        left, right = self.calculate_distance_to_goal()
        return (abs(left), abs(right))

    def is_distance_goal_complete(self):
        abs_left, abs_right = self.calculate_abs_distance_to_goal()
        if abs_left < DISTANCE_COMPLETE_THRESHOLD and abs_right < DISTANCE_COMPLETE_THRESHOLD:
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

    def balance_wheel_velocity(self):
        cmd = WheelsCmdStamped()
        abs_left, abs_right = self.calculate_abs_distance_to_goal()

        # edge case: both wheels are not moving
        if abs_left < DISTANCE_COMPLETE_THRESHOLD and abs_right < DISTANCE_COMPLETE_THRESHOLD:
            rospy.logerr("The abs_left and abs_right are both below the threshold in balance_wheel_velocity()!")
            rospy.logerr("This should not happen!")
            cmd.vel_left = 0.0
            cmd.vel_right = 0.0
            return cmd

        # calculate the direction of the wheels
        # (positive for forward, negative for backward)
        left_direction_scalar, right_direction_scalar = self.calculate_direction_scalar()
        
        if abs_left < DISTANCE_COMPLETE_THRESHOLD: # left wheel is not moving
            cmd.vel_left = 0.0
            cmd.vel_right = WHEEL_VELOCITY * right_direction_scalar
        elif abs_right < DISTANCE_COMPLETE_THRESHOLD: # right wheel is not moving
            cmd.vel_left = WHEEL_VELOCITY * left_direction_scalar
            cmd.vel_right = 0.0
        else: # both wheels are moving
            # calculate the scaling factors for the wheel velocities
            left_scaling_factor = abs_left / abs_right
            right_scaling_factor = abs_right / abs_left

            # calculate the wheel velocities
            cmd.vel_left = WHEEL_VELOCITY * left_scaling_factor
            cmd.vel_right = WHEEL_VELOCITY * right_scaling_factor

            # clamp the velocities to a maximum and minimum value
            cmd.vel_left = max(min(cmd.vel_left, MAX_VELOCITY), MIN_VELOCITY)
            cmd.vel_right = max(min(cmd.vel_right, MAX_VELOCITY), MIN_VELOCITY)

            # wheel velocities are always positive at this point
            # multiply the velocities by the direction scalars
            # to get the correct direction
            cmd.vel_left *= left_direction_scalar
            cmd.vel_right *= right_direction_scalar

        # slow down the wheels if they are too close to the goal
        # to avoid overshooting
        if abs_left < DISTANCE_SLOWDOWN_THRESHOLD_FINAL:
            cmd.vel_left *= SLOWDOWN_FACTOR_FINAL
        elif abs_left < DISTANCE_SLOWDOWN_THRESHOLD_APPROACH:
            cmd.vel_left *= SLOWDOWN_FACTOR_APPROACH

        if abs_right < DISTANCE_SLOWDOWN_THRESHOLD_FINAL:
            cmd.vel_right *= SLOWDOWN_FACTOR_FINAL
        elif abs_right < DISTANCE_SLOWDOWN_THRESHOLD_APPROACH:
            cmd.vel_right *= SLOWDOWN_FACTOR_APPROACH

        return cmd

    def handle_distance_goal(self):
        if self.is_distance_goal_complete():
            cmd = WheelsCmdStamped()
            cmd.vel_left = 0.0
            cmd.vel_right = 0.0
            self._dist_goal_active = False
            rospy.loginfo("Distance goal complete!")
        else:
            cmd = self.balance_wheel_velocity()
        self._velocity_publisher.publish(cmd)

    def handle_square_goal(self):
        if self._square_edges_completed > 3: # completed all edges of the square
            self._square_edges_completed = 0
            self._square_straight_started = False
            self._square_straight_complete = False
            self._square_turn_started = False
            self._square_turn_complete = False
            self._square_goal_active = False
            return
        
        if not self._square_straight_started:
            self._square_straight_started = True
            self._goal_distance_publisher.publish(self._square_edge_length)

        elif self._square_straight_started and not self._square_straight_complete:
            if not self._dist_goal_active:
                self._square_straight_complete = True
        
        elif not self._square_turn_started:
            self._square_turn_started = True
            self._goal_angle_publisher.publish(math.pi / 2.0) # right angle turn

        elif self._square_turn_started and not self._square_turn_complete:
            if not self._dist_goal_active:
                self._square_edges_completed += 1
                self._square_straight_started = False
                self._square_straight_complete = False
                self._square_turn_started = False
                self._square_turn_complete = False
    
    def handle_goals(self):
        if self._dist_goal_active:
            self.handle_distance_goal()
        if self._square_goal_active:
            self.handle_square_goal()

    # def run(self):
    #     rate = rospy.Rate(TIMER_FREQUENCY)
    #     while not rospy.is_shutdown():
    #         if self._new_wheel_movement_info:
    #             self._new_wheel_movement_info = False
    #             if self._dist_goal_active:
    #                 self.handle_distance_goal()
    #             if self._square_goal_active:
    #                 self.handle_square_goal()
    #         rate.sleep()

if __name__ == '__main__':
    try:
        straight_turns_squares_class_instance = StraightsTurnsSquares()
        #straight_turns_squares_class_instance.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
