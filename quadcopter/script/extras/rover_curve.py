#!/usr/bin/env python
import rospy
import tf
import scipy.linalg as la
import numpy as np
from math import *
import mavros_msgs.srv
from mavros_msgs.msg import AttitudeTarget
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import *
from test.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from quadcopter.msg import *
import time

rospy.init_node('curve', anonymous=True)
pub = rospy.Publisher("/rover/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)

msg = Twist()

def callback(info):
    global msg
    ###     Positions in global gazebo frame
    x = info.pose.pose.position.y
    y = -info.pose.pose.position.x
    z = info.pose.pose.position.z

    ###     Orientations in global of mavros frame
    a1 = info.pose.pose.orientation.x
    b1 = info.pose.pose.orientation.y
    c1 = info.pose.pose.orientation.z
    d1 = info.pose.pose.orientation.w

    roll, pitch, yaw = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])

    ###     Orientations in gazebo frame
    yaw = yaw-np.pi/2
    if yaw<np.pi/2:
        yaw = yaw+2*np.pi/2
    if yaw>np.pi/2:
        yaw = yaw-2*np.pi/2

    if 0<abs(y)<2 and 0<abs(x)<2:
        msg.linear.x = 0.8
        msg.angular.z = 0.4
        rospy.loginfo("1")
    if 0<abs(y)<2 and 1.9<abs(x)<4:
        msg.linear.x = 0.8
        msg.angular.z = -0.4
        rospy.loginfo("2")

    if abs(y)<0.6 and 4.2<abs(x):
        msg.linear.x = 0.7
        msg.angular.z = 0.6*(0-yaw)
        rospy.loginfo("3")
    
    if abs(y)<0.4 and 4.2<abs(x):
        msg.linear.x = 0.7
        msg.angular.z = 0.3*(0-yaw)
        rospy.loginfo("4")

    #rospy.loginfo("MSG %s %s",x, y)

    pub.publish(msg)


def listener():
    rospy.Subscriber("/rover/mavros/local_position/odom", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass