#!/usr/bin/env python3

#Python Libs
import sys, time

#numpy
import numpy as np

#OpenCV
import cv2
from cv_bridge import CvBridge

#ROS Libraries
import rospy
import roslib

#ROS Message Types
from sensor_msgs.msg import CompressedImage

class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()
  
        self.image_sub = rospy.Subscriber('/vader/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)

        rospy.init_node("my_lane_detector")

    def image_callback(self,msg):
        rospy.loginfo("image_callback")


        # Convert to opencv image 
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        #### YOUR CODE GOES HERE ####

        # flip along the horizontal axis using an OpenCV function
        img_out = cv2.flip(img, 0)




        #############################

        # Show image in a window
        cv2.imshow('img_out',img_out)
        cv2.waitKey(1)

    def run(self):
    	rospy.spin() # Spin forever but listen to message callbacks

if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass
    
    
