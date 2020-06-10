#!/usr/bin/env python
import rospy
import tf
import numpy as np
from math import *
from std_msgs.msg import Float64   
from geometry_msgs.msg import TwistStamped, Twist
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, HomePosition
from nav_msgs.msg import Odometry
import utm

rospy.init_node('waypoint', anonymous=True)
pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)


receive = [0.0,0.0]
error_ang_sum = 0.0
error_ang_prev = 0.0
error_dist_prev = 0.0
error_dist_sum = 0.0
def fn(info):
    rospy.loginfo("IN LIS2")
    global receive
    receive[0] = info.geo.latitude
    receive[1] = info.geo.longitude
    receive = list(receive)
    receive = utm.from_latlon(receive[0],receive[1])

go_to_posn_latlong_full = [[-35.362655,149.164046],[-35.360847,149.165078],[-35.360002,149.162881],[-35.363281,149.165263]]
pos = 0
num = 0
d = []


def get_dist(x,y):
    a = utm.from_latlon(x[0],x[1])
    a = list(a)
    a[0] = a[0] - y[0]
    a[1] = a[1] - y[1]
    return a

def callback(data):
    global receive
    global num
    global go_to_posn_latlong_full
    global error_ang_sum
    global error_ang_prev
    global error_dist_prev
    global error_dist_sum
    global pos
    ang1 =  data.pose.pose.orientation.x
    ang2 =  data.pose.pose.orientation.y
    ang3 =  data.pose.pose.orientation.z
    ang4 =  data.pose.pose.orientation.w
    l = [ang1,ang2,ang3,ang4]
    euler = tf.transformations.euler_from_quaternion(l)
    euler = list(euler)

    if pos < len(go_to_posn_latlong_full):
        go_to_posn_latlong = go_to_posn_latlong_full[pos]

        go_to_posn_xy = get_dist(go_to_posn_latlong_full[pos],receive)

        #go_to_posn_xy = utm.from_latlon(go_to_posn_latlong[0],go_to_posn_latlong[1])
        #go_to_posn_xy =  list(go_to_posn_xy)
        #go_to_posn_xy[0] = go_to_posn_xy[0] - receive[0]
        #go_to_posn_xy[1] = go_to_posn_xy[1] - receive[1]



        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        error_dist = sqrt((go_to_posn_xy[1]-y)**2 + (go_to_posn_xy[0]-x)**2)

        if error_dist<1 and pos<len(go_to_posn_latlong_full)-1:
            pos+=1
            go_to_posn_latlong = go_to_posn_latlong_full[pos]
            go_to_posn_xy = get_dist(go_to_posn_latlong_full[pos],receive)
            error_dist = sqrt((go_to_posn_xy[1]-y)**2 + (go_to_posn_xy[0]-x)**2)
        

        #z = data.pose.position.
        head_reqd = atan2((go_to_posn_xy[1]-y),(go_to_posn_xy[0]-x))

        if head_reqd>np.pi:
            head_reqd = head_reqd - 2*np.pi
        if head_reqd<-np.pi:
            head_reqd = head_reqd + 2*np.pi

        if euler[2]>np.pi:
            euler[2] = euler[2] - 2*np.pi
        if euler[2]<-np.pi:
            euler[2] = euler[2] + 2*np.pi

        error_ang = euler[2]-head_reqd
        if error_ang>np.pi:
            error_ang = error_ang - 2*np.pi
        if error_ang<-np.pi:
            error_ang = error_ang + 2*np.pi

        
        msg = Twist()
        if (abs(error_ang)>=0.02*np.pi and error_dist>=0.5) and (not(pos==len(go_to_posn_latlong_full)-1 and error_dist<=2)):
            msg.linear.x = 1.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            if error_ang<-0.02*np.pi:
                rospy.loginfo(rospy.get_caller_id() + "In %s", 0)
                msg.angular.z = 1*abs(error_ang) + 0*abs(error_ang_sum) + 0.2*(abs(error_ang)-abs(error_ang_prev))
            elif error_ang>0.02*np.pi:
                rospy.loginfo(rospy.get_caller_id() + "In %s", 1)
                msg.angular.z = -(1*abs(error_ang) + 0*abs(error_ang_sum) + 0.2*(abs(error_ang)-abs(error_ang_prev)))
        
        elif abs(error_ang)<=0.2*np.pi and error_dist>=0.5:
            msg.linear.x = 12*error_dist + 10*error_dist_prev + 0* error_dist_sum
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.z = 0.0

            error_dist_prev = 0
            error_dist_sum = 0
        if pos == len(go_to_posn_latlong_full)-1 and error_dist<=2:
            msg.linear.x = 0.5*error_dist + 0*error_dist_prev + 0* error_dist_sum
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.z = 0.0

        rospy.loginfo(rospy.get_caller_id() + "I heard %s %s %s", error_dist, error_ang, pos)
        
        rate = rospy.Rate(10) # 10hz
        #msg.linear.x = 1.0
        #msg.linear.y = 0.0
        #msg.linear.z = 0.0
        #msg.angular.z = 50.0
        pub.publish(msg)

        error_dist_prev = error_dist
        error_dist_sum += error_dist
        error_ang_prev = error_ang
        error_ang_sum += error_ang


def listener():
    rospy.Subscriber("/mavros/home_position/home", HomePosition, fn)
    rospy.Subscriber("/mavros/local_position/odom", Odometry, callback)
    rospy.spin()
    

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass