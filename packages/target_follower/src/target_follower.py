#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

SEEK_ANGULAR_VELOCITY = -2.0
FOLLOW_ANGULAR_VELOCITY = 2.0
FOLLOW_ANGULAR_VELOCITY_MAX = 3.0
FOLLOW_ANGULAR_VELOCITY_MIN = 1.0
FOLLOW_ANGULAR_PROPORTIONAL_SCALAR = 50.0
FOLLOW_X_DISTANCE_THRESHOLD = 0.005 # 5mm

class Target_Follower:
    def __init__(self):
        
        #Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)

        self.cmd_vel_pub = rospy.Publisher('/vader/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/vader/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        rospy.loginfo("Target follower node started!")
        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        self.move_robot(msg.detections)
 
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

    def seek_object(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.omega = SEEK_ANGULAR_VELOCITY
        cmd_msg.v = 0.0
        return cmd_msg

    # The larger the distance, the larger the velocity
    def calculate_abs_proportional_follow_velocity(self, x):
        vel = FOLLOW_ANGULAR_VELOCITY * abs(x) * FOLLOW_ANGULAR_PROPORTIONAL_SCALAR
        # Clamp the velocity to a maximum value
        if vel > FOLLOW_ANGULAR_VELOCITY_MAX:
            vel = FOLLOW_ANGULAR_VELOCITY_MAX
        elif vel < FOLLOW_ANGULAR_VELOCITY_MIN:
            vel = FOLLOW_ANGULAR_VELOCITY_MIN
        return vel

    def calculate_follow_velocity(self, x):
        # If the object is too close, stop moving
        if abs(x) < FOLLOW_X_DISTANCE_THRESHOLD:
            return 0.0
        
        vel = self.calculate_abs_proportional_follow_velocity(x)
        if x < 0.0: # Object is to the left of the robot
            vel = -vel
        return vel

    def follow_object(self, x):
        cmd_msg = Twist2DStamped()
        cmd_msg.v = 0.0
        cmd_msg.omega = self.calculate_follow_velocity(x)
        return cmd_msg

    def move_robot(self, detections):
        cmd_msg = Twist2DStamped()
        if len(detections) == 0: # No object detected
            cmd_msg = self.seek_object()
            rospy.loginfo("No object detected. Seeking with angular velocity: %f", cmd_msg.omega)
        else: # Object detected
            x = detections[0].transform.translation.x
            y = detections[0].transform.translation.y
            z = detections[0].transform.translation.z
            rospy.loginfo("x,y,z: %f %f %f", x, y, z)
            cmd_msg = self.follow_object(x)
            rospy.loginfo("Following object with angular velocity: %f", {cmd_msg.omega})

        cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass