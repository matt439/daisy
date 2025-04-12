#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Int8

OBSTACLE_DISTANCE_THRESHOLD = 0.5  # meters, threshold distance to consider an obstacle

class ObstacleDetector:
    def __init__(self):
        # Initialize the node
        rospy.init_node('obstacle_detector_node', anonymous=True)

        # subscriber to the tof sensor topic
        self.sub_tof = rospy.Subscriber("/tof_sensor/range", Range, self.callback_tof)

        # publisher to publish the obstacle detection status
        self.pub_obstacle = rospy.Publisher('/obstacle_detection', Int8, queue_size=10)

        self.obstacle_detected = False

        # Printing to the terminal, ROS style
        rospy.loginfo("Initalized node!")

    def callback_tof(self, msg):
        # Check if the distance is less than the threshold
        if msg.range < OBSTACLE_DISTANCE_THRESHOLD:
            self.obstacle_detected = True
            rospy.loginfo("Obstacle detected!")
        else:
            self.obstacle_detected = False

        # Publish the obstacle detection status
        self.pub_obstacle.publish(self.convert_to_int8(self.obstacle_detected))

    def convert_to_int8(self, value):
        if value:
            return 1
        else:
            return 0
        
if __name__ == '__main__':
    try:
        obstacle_detector = ObstacleDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
