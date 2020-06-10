#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import math
import numpy as np
from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from quadcopter.msg import TargetInfo
from quadcopter.msg import Contour
from geometry_msgs.msg import Point32
from timeit import default_timer as timer
from nav_msgs.msg import Odometry

rospy.init_node('graphing', anonymous=True)
graph_pub = rospy.Publisher("/rover", Odometry, queue_size=10)
graph_msg = Odometry()

def graph(info):
    graph_msg.header = info.header
    graph_msg.pose.pose.position.x = info.pose.pose.position.y+8
    graph_msg.pose.pose.position.y = -info.pose.pose.position.x
    graph_msg.pose.pose.position.z = info.pose.pose.position.z

    graph_msg.twist.twist.linear.x = info.twist.twist.linear.x
    graph_msg.twist.twist.linear.y = info.twist.twist.linear.y
    graph_msg.twist.twist.linear.z = info.twist.twist.linear.z

    graph_pub.publish(graph_msg)


def listener():
    rospy.Subscriber("/rover/mavros/local_position/odom", Odometry, graph)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass