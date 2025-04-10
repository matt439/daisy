#!/usr/bin/env python3

import rospy
import os
import time
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from duckietown_msgs.msg import WheelEncoderStamped

FREQUENCY = 10  # Hz

class WheelMovementInfo:
    def __init__(self):
        # Initialize the node
        rospy.init_node('wheel_movement_info_node', anonymous=True)

        self._left_distance = 0.0
        self._left_displacement = 0.0
        self._left_velocity = 0.0

        self._right_distance = 0.0
        self._right_displacement = 0.0
        self._right_velocity = 0.0

        self._prev_time = time.time()

        # subscribers to the left and right wheel encoder topics
        self.sub_left = rospy.Subscriber("vader/left_wheel_encoder_node/tick", WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber("vader/right_wheel_encoder_node/tick", WheelEncoderStamped, self.callback_right)

        self.movement_info_publisher = rospy.Publisher('/wheel_movement_info', Float64MultiArray, queue_size=10)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initalized node!")

    def callback_left(self, msg):
        prev_left = self._left_distance
        # calculate distance travelled by left wheel
        self._left_distance += msg.data / msg.resolution
        # calculate displacement of left wheel
        self._left_displacement = self._left_distance - prev_left

    def callback_right(self, msg):
        prev_right = self._right_distance
        # calculate distance travelled by right wheel
        self._right_distance += msg.data / msg.resolution
        # calculate displacement of right wheel
        self._right_displacement = self._right_distance - prev_right

    def calculate_velocity(self):
        # calculate the time elapsed since the last update
        curr_time = time.time()
        dt = curr_time - self._prev_time
        self._prev_time = curr_time

        if dt == 0.0:
            # log a warning if dt is zero to avoid division by zero
            rospy.logwarn("Time difference is zero, skipping velocity calculation")
            return

        # calculate the velocity of both wheels
        self._left_velocity = self._left_displacement / dt
        self._right_velocity = self._right_displacement / dt

    def run(self):
        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():
            self.calculate_velocity()

            dim = MultiArrayDimension()
            dim.label = "wheel_movement_info"
            dim.size = 6  # number of elements in the array
            dim.stride = 1 # stride is 1 since we are using a 1D array
            
            layout = MultiArrayLayout()
            layout.dim = [dim]
            layout.data_offset = 0 # no offset since we are using a 1D array

            msg = Float64MultiArray()
            msg.layout = layout
            # publish the distance, displacement, and velocity of both wheels
            # as a Float64MultiArray message
            # format: [distance_left, displacement_left, velocity_left, distance_right, displacement_right, velocity_right]
            msg.data = [
                self._left_distance, self._left_displacement, self._left_velocity,
                self._right_distance, self._right_displacement, self._right_velocity
            ]
            self.movement_info_publisher.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        wheel_distance = WheelMovementInfo()
        wheel_distance.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass