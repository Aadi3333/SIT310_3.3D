#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math
import random

class TurtlesimStraightsAndTurns:
    def __init__(self):
        # Initialize class variables
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        self.dist_goal_active = False
        self.angle_goal_active = False
        self.goal_x = 0
        self.goal_y = 0
        self.goal_theta = 0

        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Initialize subscribers  
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)

        # Initialize publishers
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Initialize a timer. The timer callback will act as our main function
        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initialized node!")

        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def goal_distance_callback(self, msg):
        goal_distance = msg.data
        self.dist_goal_active = True
        # Adjusting the sign of distance based on its sign
        sign = 1 if goal_distance >= 0 else -1
        self.goal_x = self.current_x + abs(goal_distance) * math.cos(self.current_theta) * sign
        self.goal_y = self.current_y + abs(goal_distance) * math.sin(self.current_theta) * sign

    def goal_angle_callback(self, msg):
        self.angle_goal = msg.data
        self.angle_goal_active = True
        # Adjusting the sign of angle based on its sign
        self.goal_theta = self.current_theta + self.angle_goal
        self.goal_theta = self.goal_theta % (2 * math.pi)  # Ensure angle is within [0, 2*pi)

        if self.goal_theta > math.pi:  # Convert to negative if angle is greater than pi
            self.goal_theta -= 2 * math.pi

    def timer_callback(self, event):
        if self.dist_goal_active:
            dist_to_goal = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)
            if dist_to_goal > 0.01:
                twist_msg = Twist()
                # Use the sign of goal distance to determine direction
                twist_msg.linear.x = 1 if (self.goal_x - self.current_x) > 0 else -1
                self.velocity_publisher.publish(twist_msg)
            else:
                twist_msg = Twist()
                self.velocity_publisher.publish(twist_msg)
                self.dist_goal_active = False

        if self.angle_goal_active:
            angle_diff = self.goal_theta - self.current_theta
            if abs(angle_diff) > 0.01:
                twist_msg = Twist()
                # Adjusting the sign of angular velocity based on angle difference
                twist_msg.angular.z = 0.5 if angle_diff > 0 else -0.5
                self.velocity_publisher.publish(twist_msg)
            else:
                twist_msg = Twist()
                self.velocity_publisher.publish(twist_msg)
                self.angle_goal_active = False

if __name__ == '__main__': 
    try: 
        turtlesim_straights_and_turns_class_instance = TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException: 
        pass

