#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64MultiArray, Float64
from duckietown_msgs.msg import WheelsCmdStamped

TIMER_FREQUENCY = 50  # Hz
AXLE_LENGTH = 0.1  # meters
WHEEL_VELOCITY = 0.5  # m/s

class StraightsTurnsSquares:
    def __init__(self):
        # Initialize class variables
        self._last_distance_left = 0.0
        self._last_displacement_left = 0.0
        self._last_velocity_left = 0.0
        self._last_distance_right = 0.0
        self._last_displacement_right = 0.0
        self._last_velocity_right = 0.0
        self._new_wheel_movement_info = False

        self._goal_distance_left = 0.0
        self._goal_displacement_left = 0.0
        self._goal_distance_right = 0.0
        self._goal_displacement_right = 0.0
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

        # rospy.Timer(rospy.Duration(TIMER_PERIOD), self.timer_callback)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initalized node!")

    def wheel_movement_info_callback(self, msg):
        self._last_distance_left = msg.data[0]
        self._last_displacement_left = msg.data[1]
        self._last_velocity_left = msg.data[2]
        self._last_distance_right = msg.data[3]
        self._last_displacement_right = msg.data[4]
        self._last_velocity_right = msg.data[5]
        self._new_wheel_movement_info = True

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
        self._goal_displacement_left = left_distance
        self._goal_distance_right = self._last_distance_right + right_distance
        self._goal_displacement_right = right_distance
        self.dist_goal_active = True

    def goal_distance_callback(self, msg):
        rospy.loginfo("Received goal distance: %s", msg.data)
        if msg.data == 0.0:
            return
        self._goal_distance_left = self._last_distance_left + msg.data
        self._goal_displacement_left = msg.data
        self._goal_distance_right = self._last_distance_right + msg.data
        self._goal_displacement_right = msg.data
        self.dist_goal_active = True

    def square_callback(self, msg):
        rospy.loginfo("Received square edge length: %s", msg.data)
        self._edges_completed = 0
        self._square_straight_started = False
        self._square_straight_complete = False
        self._square_turn_started = False
        self._square_turn_complete = False
        self._square_edge_length = msg.data
        self._square_goal_active = True

    def calculate_distance_to_goal(self):
        self._goal_distance_left = self._goal_distance_left - self._last_distance_left
        self._goal_distance_right = self._goal_distance_right - self._last_distance_right
        return self._goal_distance_left, self._goal_distance_right

    def calculate_abs_distance_to_goal(self):
        distance_to_goal = self.calculate_distance_to_goal()
        abs_distance_to_goal = (abs(distance_to_goal[0]), abs(distance_to_goal[1]))
        return abs_distance_to_goal

    def distance_goal_complete(self):
        abs_distance_to_goal = self.calculate_abs_distance_to_goal()
        abs_left = abs_distance_to_goal[0]
        abs_right = abs_distance_to_goal[1]

        if abs_left >= abs(self._goal_displacement_left) and abs_right >= abs(self._goal_displacement_right):
            return True
        return False
    
    def moving_forward(self):
        left_forward = self._goal_displacement_left > 0.0
        right_forward = self._goal_displacement_right > 0.0
        return (left_forward, right_forward)
    
    def moving_forward_scalar(self):
        left_forward, right_forward = self.moving_forward()
        left_scalar = 1.0 if left_forward else -1.0
        right_scalar = 1.0 if right_forward else -1.0
        return (left_scalar, right_scalar)

    def balance_wheel_velocity(self):
        abs_distance_to_goal = self.calculate_abs_distance_to_goal()
        abs_left = abs_distance_to_goal[0]
        abs_right = abs_distance_to_goal[1]
        left_scalar, right_scalar = self.moving_forward_scalar()
        left_velocity = left_scalar * WHEEL_VELOCITY
        right_velocity = right_scalar * WHEEL_VELOCITY

        cmd = WheelsCmdStamped()
        if abs_left > abs_right: # left wheel is further from goal
            cmd.vel_left = left_velocity * (abs_left / abs_right)
            cmd.vel_right = right_velocity
        else: # right wheel is further from goal
            cmd.vel_left = left_velocity
            cmd.vel_right = right_velocity * (abs_right / abs_left)
        return cmd

    def handle_distance_goal(self):
        cmd = WheelsCmdStamped()
        if self.distance_goal_complete():
            cmd.vel_left = 0.0
            cmd.vel_right = 0.0
            self.dist_goal_active = False
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
            self._goal_angle_publisher.publish(math.pi / 2.0) # right turn

        elif self._square_turn_started and not self._square_turn_complete:
            if not self.dist_goal_active:
                self._square_edges_completed += 1
                self._square_straight_started = False
                self._square_straight_complete = False
                self._square_turn_started = False
                self._square_turn_complete = False

    def run(self):
        rate = rospy.Rate(TIMER_FREQUENCY)
        while not rospy.is_shutdown():
            if self._new_wheel_movement_info:
                self._new_wheel_movement_info = False
                if self._dist_goal_active:
                    self.handle_distance_goal()
                if self._square_goal_active:
                    self.handle_square_goal()
            rate.sleep()

if __name__ == '__main__':
    try:
        straight_turns_squares_class_instance = StraightsTurnsSquares()
        straight_turns_squares_class_instance.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    