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

CROP_LEFT = 0
CROP_RIGHT = 640
CROP_TOP = 0
CROP_BOTTOM = 310
# masks are in BGR format
UPPER_WHITE_MASK = np.array([255, 255, 255])
LOWER_WHITE_MASK = np.array([170, 170, 170])
UPPER_YELLOW_MASK = np.array([55, 255, 255])
LOWER_YELLOW_MASK = np.array([0, 100, 100])
# HSV masks. # Hue: 0-179 (not 0-359), Saturation: 0-255, Value: 0-255
UPPER_WHITE_MASK_HSV = np.array([179, 25, 255])
LOWER_WHITE_MASK_HSV = np.array([0, 0, 170])
UPPER_YELLOW_MASK_HSV = np.array([30, 255, 255])
LOWER_YELLOW_MASK_HSV = np.array([15, 140, 140])
CANNY_TLOWER = 50
CANNY_TUPPER = 150
CANNY_APERTURE_SIZE = 3
CANNY_L2_GRADIENT = True
HOUGH_RHO = 1
HOUGH_THETA = np.pi/180
HOUGH_THRESHOLD = 50
HOUGH_MIN_LINE_LENGTH = 50
HOUGH_MAX_LINE_GAP = 10
ERODE_KERNEL = np.ones((5, 5), np.uint8)
DILATE_KERNEL = np.ones((5, 5), np.uint8)
ERODE_ITERATIONS = 1
DILATE_ITERATIONS = 1

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
        
        # flip along the horizontal axis using an OpenCV function
        img_out = cv2.flip(img, 0)

        # Crops the input image so that only the road is visible (roughly)
        img_cropped = img_out[CROP_TOP:CROP_BOTTOM, CROP_LEFT:CROP_RIGHT]

        # Converts the cropped image to an HSV Color Space (For demonstration purposes)
        img_cropped_hsv = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2HSV)

        # Apply color filtering for White pixels so the lane markers are visible in the
        # output image.
        mask_white = cv2.inRange(img_cropped, LOWER_WHITE_MASK, UPPER_WHITE_MASK)
        img_white = cv2.bitwise_and(img_cropped, img_cropped, mask=mask_white)
        img_white_dilated = self.dilate(img_white)
        img_white_eroded = self.erode(img_white_dilated)

        mask_white_hsv = cv2.inRange(img_cropped_hsv, LOWER_WHITE_MASK_HSV, UPPER_WHITE_MASK_HSV)
        img_white_hsv = cv2.bitwise_and(img_cropped_hsv, img_cropped_hsv, mask=mask_white_hsv)
        img_white_hsv_dilated = self.dilate(img_white_hsv)
        img_white_hsv_eroded = self.erode(img_white_hsv_dilated)

        # Apply color filtering for Yellow pixels so the dashed lines in the middle of a
        # lane are visible in the output image
        mask_yellow = cv2.inRange(img_cropped, LOWER_YELLOW_MASK, UPPER_YELLOW_MASK)
        img_yellow = cv2.bitwise_and(img_cropped, img_cropped, mask=mask_yellow)
        img_yellow_dilated = self.dilate(img_yellow)
        #img_yellow_eroded = self.erode(img_yellow_dilated) get better results without erode

        mask_yellow_hsv = cv2.inRange(img_cropped_hsv, LOWER_YELLOW_MASK_HSV, UPPER_YELLOW_MASK_HSV)
        img_yellow_hsv = cv2.bitwise_and(img_cropped_hsv, img_cropped_hsv, mask=mask_yellow_hsv)
        img_yellow_hsv_dilated = self.dilate(img_yellow_hsv)

        # convert img_white_eroded to grayscale
        img_white_gray = cv2.cvtColor(img_white_eroded, cv2.COLOR_BGR2GRAY)
        # convert img_yellow_eroded to grayscale
        img_yellow_gray = cv2.cvtColor(img_yellow_dilated, cv2.COLOR_BGR2GRAY)

        # Apply Canny Edge Detection to the White-filtered image
        img_white_canny = cv2.Canny(img_white_gray, CANNY_TLOWER, CANNY_TUPPER, apertureSize=CANNY_APERTURE_SIZE, 
                              L2gradient=CANNY_L2_GRADIENT)
        
        # Apply Canny Edge Detection to the Yellow-filtered image
        img_yellow_canny = cv2.Canny(img_yellow_gray, CANNY_TLOWER, CANNY_TUPPER, apertureSize=CANNY_APERTURE_SIZE, 
                               L2gradient=CANNY_L2_GRADIENT)

        # Apply Hough Transform to the White-filtered image
        hough_lines_white = cv2.HoughLinesP(img_white_canny, HOUGH_RHO, HOUGH_THETA, HOUGH_THRESHOLD, 
                                            None, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP)

        # Apply Hough Transform to the Yellow-filtered image
        hough_lines_yellow = cv2.HoughLinesP(img_yellow_canny, HOUGH_RHO, HOUGH_THETA, HOUGH_THRESHOLD, 
                                             None, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP)

        # Draw lines found on both Hough Transforms on the cropped image using OpenCV functions
        img_cropped_with_white_hough_lines = self.output_lines(img_cropped, hough_lines_white)
        img_cropped_with_yellow_hough_lines = self.output_lines(img_cropped, hough_lines_yellow)

        # Show image in a window
        # cv2.imshow('img_cropped',img_cropped)
        cv2.imshow('img_white_eroded', img_white_eroded)
        cv2.imshow('img_white_hsv_eroded', cv2.cvtColor(img_white_hsv_eroded, cv2.COLOR_HSV2BGR))
        cv2.imshow('img_yellow_dilated', img_yellow_dilated)
        cv2.imshow('img_yellow_hsv_dilated', cv2.cvtColor(img_yellow_hsv_dilated, cv2.COLOR_HSV2BGR))
        # cv2.imshow('img_cropped_with_white_hough_lines', img_cropped_with_white_hough_lines)
        # cv2.imshow('img_cropped_with_yellow_hough_lines', img_cropped_with_yellow_hough_lines)
        cv2.waitKey(1)

    def erode(self, img):
        img_ero = cv2.erode(img, ERODE_KERNEL, iterations = ERODE_ITERATIONS)
        return img_ero
    
    def dilate(self, img):
        img_dil = cv2.dilate(img, DILATE_KERNEL, iterations = DILATE_ITERATIONS)
        return img_dil

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
    
    
