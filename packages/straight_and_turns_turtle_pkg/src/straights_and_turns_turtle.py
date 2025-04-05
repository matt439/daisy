#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import time 
import math

class TurtlesimStraightsAndTurns:
    def __init__(self):
        
        # Initialize class variables
        self.last_distance = 0
        self.goal_distance = 0
        self.dist_goal_active = False
        self.forward_movement = True

        self.goal_rotation = 0
        self.angle_goal_active = False
        self.clockwise_rotation = True
        self.last_theta = 0
        self.total_rotation = 0

        self.position_goal_active = False
        self.last_position = (0, 0)
        self.rotation_started = False
        self.rotation_complete = False
        self.distance_started = False
        self.distance_complete = False
        self.position_goal_distance = 0
        self.position_goal_angle = 0

        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Initialize subscribers  
        rospy.Subscriber("/turtle_dist", Float64,self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64,self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64,self.goal_distance_callback)
        rospy.Subscriber("/goal_position", Point, self.goal_position_callback)
        rospy.Subscriber("/turtle1/pose", Pose,self.pose_callback)

        # Initialize publishers
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.goal_angle_publisher = rospy.Publisher('/goal_angle', Float64, queue_size=10)
        self.goal_distance_publisher = rospy.Publisher('/goal_distance', Float64, queue_size=10)

        # Initialize a timer. The timer callback will act as our main function
        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initalized node!")
        
        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    def pose_callback(self,msg):
        self.total_rotation += abs(self.angle_difference(self.last_theta, msg.theta))
        self.last_theta = msg.theta
        self.last_position = (msg.x, msg.y)

    def distance_callback(self,msg):
        self.last_distance = msg.data

    def angle_difference(self, angle_1, angle_2):
        diff = angle_1 - angle_2
        while diff < -math.pi:
            diff += 2 * math.pi
        while diff > math.pi:
            diff -= 2 * math.pi
        return diff

    def goal_position_callback(self,msg):
        if msg.x < 0:
            msg.x = 0
        if msg.y < 0:
            msg.y = 0
        
        self.position_goal_active = True
        self.rotation_started = False
        self.rotation_complete = False
        self.distance_started = False
        self.distance_complete = False

        diff_vector = (msg.x - self.last_position[0], msg.y - self.last_position[1])
        diff_vector_angle = math.atan2(diff_vector[1], diff_vector[0])
        self.position_goal_angle = self.angle_difference(diff_vector_angle, self.last_theta)
        self.position_goal_distance = (diff_vector[0]**2 + diff_vector[1]**2)**0.5

    def goal_angle_callback(self,msg):
        if msg.data == 0.0:
            return
        elif msg.data < 0:
            self.clockwise_rotation = True
            self.goal_rotation = self.total_rotation - msg.data
        else:
            self.clockwise_rotation = False
            self.goal_rotation = self.total_rotation + msg.data

        self.angle_goal_active = True

    def goal_distance_callback(self,msg):
        # Set goal_distance, dist_goal_active and forward_movement variables here
        if msg.data == 0.0:
            return
        elif msg.data < 0:
            self.forward_movement = False
            self.goal_distance = self.last_distance - msg.data
        else:
            self.forward_movement = True
            self.goal_distance = self.last_distance + msg.data
        
        self.dist_goal_active = True

    def handle_distance_goal(self):
        linear_cmd = Twist()
        
        if self.last_distance >= self.goal_distance:
            linear_cmd.linear.x = 0.0
            self.dist_goal_active = False
        elif self.forward_movement:
            linear_cmd.linear.x = 2.0
        else:
            linear_cmd.linear.x = -2.0

        self.velocity_publisher.publish(linear_cmd)

    def handle_angle_goal(self):
        angle_cmd = Twist()

        if self.total_rotation >= self.goal_rotation:
            angle_cmd.angular.z = 0.0
            self.angle_goal_active = False
        elif self.clockwise_rotation:
            angle_cmd.angular.z = -2.0
        else:
            angle_cmd.angular.z = 2.0

        self.velocity_publisher.publish(angle_cmd)

        #print("handle angle goal")

    def hangle_position_goal(self):
        if not self.rotation_started:
            self.goal_angle_publisher.publish(self.position_goal_angle)
            self.rotation_started = True

        elif self.rotation_started and not self.rotation_complete:
            if not self.angle_goal_active:
                self.rotation_complete = True

        elif not self.distance_started:
            self.goal_distance_publisher.publish(self.position_goal_distance)
            self.distance_started = True
            
        elif self.distance_started and not self.distance_complete:
            if not self.dist_goal_active:
                print(self.last_position)
                self.distance_complete = True
                self.position_goal_active = False

    def timer_callback(self,msg):
        # If a goal is active, first check if the goal is reached (it's OK if the goal is not perfectly reached)
        # Then publish a cmd_vel message
        if self.dist_goal_active:
            self.handle_distance_goal()
        if self.angle_goal_active:
            self.handle_angle_goal()
        if self.position_goal_active:
            self.hangle_position_goal()


if __name__ == '__main__': 

    try: 
        turtlesim_straights_and_turns_class_instance = TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException: 
        pass
        
