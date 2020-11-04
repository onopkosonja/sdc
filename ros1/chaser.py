#! /usr/bin/python
import rospy
import numpy as np
from rospy import Publisher, Subscriber
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class Chaser:
    def __init__(self, vel):
        self.vel = vel
        self.subsc1 = Subscriber('/turtle1/pose', Pose, self.get_pose1)
        self.subsc2 = Subscriber('/turtle2/pose', Pose, self.get_pose2)
        self.pub = Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        self.pose2 = Pose()

    def get_pose1(self, pose):
        self.pose2 = pose

    def get_pose2(self, pose):
        new_angle = np.arctan2(x1=[self.pose2.y, pose.y], x2=[self.pose2.x, pose.x])

        while new_angle < - np.pi:
            new_angle += 2 * np.pi
        while new_angle > np.pi:
            new_angle -= 2 * np.pi

        msg = Twist()
        msg.linear.x = self.vel
        msg.angular.z = new_angle
        self.pub.publish(msg)


rospy.init_node('chaser')
vel = float(rospy.get_param('~velocity'))
Chaser(vel)
rospy.spin()
