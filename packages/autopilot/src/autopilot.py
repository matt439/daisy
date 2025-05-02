#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Autopilot:
    def __init__(self):
        
        #Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        self.cmd_vel_pub = rospy.Publisher('/vader/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/vader/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/vader/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        # Set parameters for the line detector
        rospy.set_param('/vader/line_detector_node/colors/WHITE/high', [180, 25, 255])
        rospy.set_param('/vader/line_detector_node/colors/WHITE/low', [0, 0, 200])
        rospy.set_param('/vader/line_detector_node/colors/YELLOW/high', [30, 255, 255])
        rospy.set_param('/vader/line_detector_node/colors/YELLOW/low', [15, 140, 160])

        rospy.set_param('/vader/line_detector_node/img_size', [125, 165])

        rospy.set_param('/vader/line_detector_node/line_detector_parameters/canny_aperture_size', 3)
        rospy.set_param('/vader/line_detector_node/line_detector_parameters/canny_thresholds', [80, 200])
        rospy.set_param('/vader/line_detector_node/line_detector_parameters/dilation_kernel_size', 3)
        rospy.set_param('/vader/line_detector_node/line_detector_parameters/hough_max_line_gap', 5)
        rospy.set_param('/vader/line_detector_node/line_detector_parameters/hough_min_line_length', 10)
        rospy.set_param('/vader/line_detector_node/line_detector_parameters/hough_threshold', 10)

        rospy.set_param('/vader/line_detector_node/top_cutoff', 41)

        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING":
            return
        
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

    def set_state(self, state):
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)

    def move_robot(self, detections):

        #### YOUR CODE GOES HERE ####

        if len(detections) == 0:
            return

        # self.set_state("NORMAL_JOYSTICK_CONTROL") # Stop Lane Following

        # Process AprilTag info and publish a velocity
        
        # For open loop control, consider using rospy.sleep() afterwards.
        
        #self.set_state("LANE_FOLLOWING") # Go back to lane following

        #############################

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass