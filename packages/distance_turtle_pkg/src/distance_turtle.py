#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import time 

class DistanceReader:
    def __init__(self):
        
        # Initialize the node
        rospy.init_node('turtlesim_distance_node', anonymous=True)

        # Initialize subscriber: input the topic name, message type and callback signature  
        rospy.Subscriber("/turtle1/pose", Pose,self.callback)

        # Initialize publisher: input the topic name, message type and msg queue size
        self.distance_publisher = rospy.Publisher('/turtle_dist', Float64, queue_size=10)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initalized node!")
        
        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    first_update = True
    prev_x = 0.0
    prev_y = 0.0
    distance = 0.0

    # Whenever a message is received from the specified subscriber, this function will be called
    def callback(self,msg):
        rospy.loginfo("Turtle Position: %s %s", msg.x, msg.y)

        # Calculate the distance the turtle has travelled and publish it
        if self.first_update:
            self.prev_x = msg.x
            self.prev_y = msg.y
            self.first_update = False
            return
        
        self.distance += ((msg.x - self.prev_x)**2 + (msg.y - self.prev_y)**2)**0.5
        self.distance_publisher.publish(self.distance)
        self.prev_x = msg.x
        self.prev_y = msg.y

if __name__ == '__main__': 

    try: 
        distance_reader_class_instance = DistanceReader()
    except rospy.ROSInterruptException: 
        pass
        
