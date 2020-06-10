#!/usr/bin/env python
import rospy
import tf
import scipy.linalg as la
import numpy as np
from math import *
import mavros_msgs.srv
from mavros_msgs.msg import AttitudeTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import *
from test.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from quadcopter.msg import *
import time
import control.matlab as mb

rospy.init_node('sdre', anonymous=True)
pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

roll = 0.0
pitch = 0.0
yaw = 0.0

msg = AttitudeTarget()
#[10,-10,0]    #ardu

goal = np.array([50.0, 5.0, 0.0])
goal_body = np.array([0.0, 0.0, 0.0])

x = 0.0
y = 0.0
z = 0.0

error_head_prev = 0.0

camera_mount = 0.785398
horizontal = 1.04719/2
vertical = 1.04719/2

vel_rover = [0,0,0]

A = np.array([[0, 1, 0, 0, 0, 0]
            ,[0, 0, 0, 0, 0, 0]
            ,[0, 0, 0, 1, 0, 0]
            ,[0, 0, 0, 0, 0, 0]
            ,[0, 0, 0, 0, 0, 1]
            ,[0, 0, 0, 0, 0, 0]])


now_p = time.time()

####    msg.x in mavros is -y in gazebo
def land():
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
    print "set mode: ", set_mode(208,'GUIDED')
    land = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
    print "land: ", land(0,0,0,0,0)


def callback(info):
    ##MUST GET HEADING
    global x, y, z, roll, pitch, yaw, vel_rover, vel_drone_rot, vel_drone_trans, head, now_p, error_head_prev, goal, goal_body
    
    ############################        ARDUPILOT-MAVROS COORDINATE FRAME
    ###     Positions in global frame
    x = info.pose.pose.position.x
    y = info.pose.pose.position.y
    z = info.pose.pose.position.z

    ###     Orientations in global frame
    a1 = info.pose.pose.orientation.x
    b1 = info.pose.pose.orientation.y
    c1 = info.pose.pose.orientation.z
    d1 = info.pose.pose.orientation.w

    ###     All linear velocities are local 
    v_x = info.twist.twist.linear.x
    v_y = info.twist.twist.linear.y
    v_z = info.twist.twist.linear.z

    ###     All angular velocities are local
    v_roll = info.twist.twist.angular.x
    v_pitch = info.twist.twist.angular.y
    v_yaw = info.twist.twist.angular.z

    roll, pitch, yaw = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])
    
    Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
    Rot_inertial_to_body = Rot_body_to_inertial.transpose()

    goal_body[0] = goal[0]-x
    goal_body[1] = goal[1]-y
    goal_body[2] = goal[2]-z

    #rospy.loginfo("GOA1 %s", goal_body)
    ####    Global to Body rotation
    goal_body = np.matmul(Rot_inertial_to_body,goal_body.transpose())
    #rospy.loginfo("GOAL2 %s %s", Rot_inertial_to_body, goal_body)
    #rospy.loginfo("GOAL %s", goal_body)
    

    #vel_rover = np.dot(Rot,vel_rover)       ####    Velocity transformations to be done
    #vel_rover_body = [vel_rover[1],vel_rover[0],vel_rover[2]]

    Q = np.array([[goal_body[0]**2, 0, 0, 0, 0, 0]
                ,[0, (goal_body[0]+2)**2, 0, 0, 0, 0]
                ,[0, 0, goal_body[1]**2, 0, 0, 0]
                ,[0, 0, 0, (goal_body[1]+2)**2, 0, 0]
                ,[0, 0, 0, 0, 1, 0]
                ,[0, 0, 0, 0, 0, 1]])

    R = np.array([[abs(goal_body[0]*goal_body[1]), 0, 0]
                ,[0, goal_body[0]**2, 0]
                ,[0, 0, goal_body[1]**2]])


    ###     Calculation for control done in body fixed frame

    X = np.array([[goal_body[0]],[vel_rover[0]-v_x],[goal_body[1]],[vel_rover[1]-v_y],[goal_body[2]],[vel_rover[2]-v_z]])

    ###     d2(e_x)/dt2 = 0-d2(x)/dt2 so all signs inverted
    B = np.array([[0, 0, 0], [0, -9.8, 0], [0, 0, 0], [0, 0, 9.8], [0, 0, 0], [-1, 0, 0]])

    #(P,L,G) = mb.care(A, B, Q, R)
    P = la.solve_continuous_are(A, B, Q, R)

    u = np.matmul(-np.linalg.inv(R),B.transpose())
    u = np.matmul(u,P)
    u = np.dot(u,X)
    
    #rospy.loginfo("U%s",u)

    #Rot = Rot.transpose()
    #inputs = np.dot(Rot,[[0], [0], [u[0]]])

    u[0] = (u[0]+9.8)*0.5/9.8
    if u[0]>0.5:
        u[0] = 0.5
    if u[1]>15*np.pi/180:
        u[1] = 15*np.pi/180
    if u[1]<-15*np.pi/180:
        u[1] = -15*np.pi/180
    if u[2]>15*np.pi/180:
        u[2] = 15*np.pi/180
    if u[2]<-15*np.pi/180:
        u[2] = -15*np.pi/180

    
    now = time.time()

    if z<0.25:
        land()

    else:
        quater = tf.transformations.quaternion_from_euler(u[2],u[1],yaw) #+yaw_rate*(now-now_p)
        msg.header = Header()
        msg.type_mask = 0
        msg.orientation.x = quater[0]
        msg.orientation.y = quater[1]
        msg.orientation.z = quater[2]
        msg.orientation.w = quater[3]
        msg.body_rate.x = 0.0
        msg.body_rate.y = 0.0
        msg.body_rate.z = 0.0
        msg.thrust = u[0]

        ##VELOCITIES HERE

        pub.publish(msg)
    now_p = time.time()
    #error_head_prev = error_head

    rospy.loginfo("States %s", X)
    rospy.loginfo("Inputs %s", u)
    #rospy.loginfo("ANGLES %s",[roll, pitch, yaw])

    rate = rospy.Rate(10) 

'''
def ReceiveTar(data):
    global goal, x, y, z, roll, pitch, yaw, camera_mount, horizontal, vertical
    xt_image=data.contour.center.x
    yt_image=data.contour.center.y
    xt_image -= 250
    yt_image -= 250
    width=data.contour.width
    height=data.contour.height

    if(width<30 or height<30):
        goal[0] = goal[0] + vel_rover[0]*0.1
        goal[1] = goal[1] + vel_rover[1]*0.1
        ro
        #rospy.loginfo("DATA %s %s",xt_image,yt_image)

    else:
        d_xbound = 2*(z/sin(camera_mount))*tan(horizontal)
        x_ppm = d_xbound/500

        d_ybound = z/tan(camera_mount-vertical) - z/tan(camera_mount+vertical)
        y_ppm = d_ybound/500

        x_origin = x + (z/tan(camera_mount))*cos(yaw)       #In global frame
        y_origin = y + (z/tan(camera_mount))*sin(yaw)

        yt_image = -yt_image

        xt_image = xt_image*x_ppm
        yt_image = yt_image*y_ppm

        x_new = x_origin + xt_image*cos(yaw-np.pi/2) - yt_image*sin(yaw-np.pi/2)
        y_new = y_origin + xt_image*sin(yaw-np.pi/2) + yt_image*cos(yaw-np.pi/2)

        #x_new = x - x_prev*cos(yaw)
        #y_new = y - y_prev*sin(yaw)

        goal[0] = x_new
        goal[1] = y_new

        rospy.loginfo("POSN %s %s %s %s ", x_new, y_new, x, y)
        
'''

def listener():
    rospy.Subscriber("/mavros/local_position/odom", Odometry, callback)
    #rospy.Subscriber('/landing_target_info_new', TargetInfo,ReceiveTar)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass