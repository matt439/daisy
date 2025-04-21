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

CROP_LEFT = 100
CROP_RIGHT = 540
CROP_TOP = 200
CROP_BOTTOM = 400
# masks are in BGR format
UPPER_WHITE_MASK = np.array([255, 255, 255])
LOWER_WHITE_MASK = np.array([150, 150, 150])
UPPER_YELLOW_MASK = np.array([150, 255, 255])
LOWER_YELLOW_MASK = np.array([0, 150, 150])
CANNY_TLOWER = 50
CANNY_TUPPER = 150
CANNY_APERTURE_SIZE = 3
CANNY_L2_GRADIENT = True
HOUGH_RHO = 1
HOUGH_THETA = np.pi/180
HOUGH_THRESHOLD = 50
HOUGH_MIN_LINE_LENGTH = 50
HOUGH_MAX_LINE_GAP = 10

class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()
  
        self.image_sub = rospy.Subscriber('/vader/camera_node/image/compressed', 
                                          CompressedImage, self.image_callback, queue_size=1)

        rospy.init_node("my_lane_detector")

    def image_callback(self,msg):
        rospy.loginfo("image_callback")

        # Convert to opencv image 
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        #### YOUR CODE GOES HERE ####

        # flip along the horizontal axis using an OpenCV function
        img_out = cv2.flip(img, 0)
        img_out = cv2.flip(img_out, 1)

        # Crops the input image so that only the road is visible (roughly)
        img_out = img_out[CROP_TOP:CROP_BOTTOM, CROP_LEFT:CROP_RIGHT]

        # Converts the cropped image to an HSV Color Space (For demonstration
        # purposes, convert the images back to the RGB domain after image
        # processing operations below.)
        img_out = cv2.cvtColor(img_out, cv2.COLOR_BGR2HSV)

        # Apply color filtering for White pixels so the lane markers are visible in the
        # output image.
        mask_white = cv2.inRange(img_out, LOWER_WHITE_MASK, UPPER_WHITE_MASK)
        img_white = cv2.bitwise_and(img_out, img_out, mask=mask_white)

        # Apply color filtering for Yellow pixels so the dashed lines in the middle of a
        # lane are visible in the output image
        mask_yellow = cv2.inRange(img_out, LOWER_YELLOW_MASK, UPPER_YELLOW_MASK)
        img_yellow = cv2.bitwise_and(img_out, img_out, mask=mask_yellow)

        # Apply Canny Edge Detector to the cropped image
        
        # Image: Input image to which Canny filter will be applied
        # T_lower: Lower threshold value in Hysteresis Thresholding
        # T_upper: Upper threshold value in Hysteresis Thresholding
        # aperture_size: Aperture size of the Sobel filter.
        # L2Gradient: Boolean parameter used for more precision in calculating Edge Gradient.
        img_canny = cv2.Canny(img_out, CANNY_TLOWER, CANNY_TUPPER, apertureSize=CANNY_APERTURE_SIZE, 
                              L2gradient=CANNY_L2_GRADIENT)

        # Apply Hough Transform to the White-filtered image
        # convert img_white to grayscale
        img_white_gray = cv2.cvtColor(img_white, cv2.COLOR_BGR2GRAY)
        hough_lines_white = cv2.HoughLinesP(img_white_gray, HOUGH_RHO, HOUGH_THETA, HOUGH_THRESHOLD, 
                                            None, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP)

        # Apply Hough Transform to the Yellow-filtered image
        # convert img_yellow to grayscale
        img_yellow_gray = cv2.cvtColor(img_yellow, cv2.COLOR_BGR2GRAY)
        hough_lines_yellow = cv2.HoughLinesP(img_yellow_gray, HOUGH_RHO, HOUGH_THETA, HOUGH_THRESHOLD, 
                                             None, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP)

        # Draw lines found on both Hough Transforms on the cropped image using OpenCV functions

        img_out_with_white_hough_lines = self.output_lines(img_out, hough_lines_white)
        img_out_with_yellow_hough_lines = self.output_lines(img_out, hough_lines_yellow)
        #############################

        # Show image in a window
        cv2.imshow('img_out',img_out)
        cv2.imshow('img_white',img_white)
        cv2.imshow('img_yellow',img_yellow)
        cv2.imshow('img_img_out_with_white_hough_lines',img_out_with_white_hough_lines)
        cv2.imshow('img_out_with_yellow_hough_lines',img_out_with_yellow_hough_lines)
        cv2.waitKey(1)

    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output

    def run(self):
    	rospy.spin() # Spin forever but listen to message callbacks

if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass
    
    
