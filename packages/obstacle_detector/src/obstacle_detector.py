#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Int8

OBSTACLE_DISTANCE_THRESHOLD = 0.25  # meters, threshold distance to consider an obstacle

class ObstacleDetector:
    def __init__(self):
        # Initialize the node
        rospy.init_node('obstacle_detector_node', anonymous=True)

        # subscriber to the tof sensor topic
        self.sub_tof = rospy.Subscriber("/vader/front_center_tof_driver_node/range", Range, self.callback_tof)

        # publisher to publish the obstacle detection status
        self.pub_obstacle = rospy.Publisher('/vader/obstacle_detector', Int8, queue_size=10)

        self._prev_obstacle_detected = False

        # Printing to the terminal, ROS style
        rospy.loginfo("Initalized obstacle_detector node!")

    def publish_obstacle_status(self, obstacle_detected):
        self.pub_obstacle.publish(int(obstacle_detected))

    def callback_tof(self, msg):
        obstacle_now = msg.range < OBSTACLE_DISTANCE_THRESHOLD
        if obstacle_now != self._prev_obstacle_detected:
            if obstacle_now:
                rospy.loginfo("Obstacle detected!")
            else:
                rospy.loginfo("Obstacle cleared!")
            self.publish_obstacle_status(obstacle_now)
        self._prev_obstacle_detected = obstacle_now

if __name__ == '__main__':
    try:
        obstacle_detector = ObstacleDetector()
        # no need to call run() as it relies on the callbacks from TOF to update the state
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

