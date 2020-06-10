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
import tf
import scipy.linalg as la
import scipy.signal as sig
from math import *
import mavros_msgs.srv
from mavros_msgs.msg import AttitudeTarget
from nav_msgs.msg import Odometry
from test.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
import control.matlab as mb
from timeit import default_timer as timer
from quadcopter.msg import *


rospy.init_node('final_control', anonymous=True)
pub = rospy.Publisher("/drone/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)
msg = AttitudeTarget()

now = timer()
now_p = timer()

xt_image = 0.0
yt_image = 0.0
radius = 0.0
detect = 1

x = 0.0
y = 0.0
z = 0.0

roll = 0.0
pitch = 0.0
yaw = 0.0
v_roll = 0.0
v_pitch = 0.0
v_yaw = 0.0
v_x = 0.0
v_y = 0.0
v_z = 0.0
Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                                ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                                ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
Rot_inertial_to_body = Rot_body_to_inertial.transpose()

x1_prev = 0.0
x2_prev = 0.0
v1_prev = 0.0
v2_prev = 0.0
v1 = 0.0
v2 = 0.0
goal_pred = np.array([[0.0]
                    ,[0.0]
                    ,[0.0]
                    ,[0.0]])

goal_pred_var = np.array([[1, 0, 0, 0]
                        ,[0, 1, 0, 0]
                        ,[0, 0, 1, 0]
                        ,[0, 0, 0, 1]])

cvFrame = np.zeros((500,500,3), np.uint8)
flag_imageshow=1

w1_prev = 0.0
w2_prev = 0.0
v1_prev = 0.0
v2_prev = 0.0

X = np.array([[0.0]
            ,[0.0]
            ,[0.0]
            ,[0.0]])

P = np.array([[np.random.normal(0, 1), 0, 0, 0]
            ,[0, np.random.normal(0, 0.25), 0, 0]
            ,[0, 0, np.random.normal(0, 1), 0]
            ,[0, 0, 0, np.random.normal(0, 0.25)]])

H = np.array([[1, 0, 0, 0]
            ,[0, 1, 0, 0]
            ,[0, 0, 1, 0]
            ,[0, 0, 0, 1]])

hori_fov = np.pi/6 #on either side
vert_fov = 500*hori_fov/500

goal = np.array([0.0, 0.0, 0.0])
goal_body = np.array([0.0, 0.0, 0.0])
vel_rover = [0.0,0.0,0.0]
A_sdre = np.array([[0, 1, 0, 0, 0, 0]
            ,[0, 0, 0, 0, 0, 0]
            ,[0, 0, 0, 1, 0, 0]
            ,[0, 0, 0, 0, 0, 0]
            ,[0, 0, 0, 0, 0, 1]
            ,[0, 0, 0, 0, 0, 0]])


def segment_colour(frame):
    hsv_roi =  cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_1 = cv2.inRange(hsv_roi, np.array([160, 160,10]), np.array([190,255,255]))
    ycr_roi=cv2.cvtColor(frame,cv2.COLOR_BGR2YCrCb)
    mask_2=cv2.inRange(ycr_roi, np.array((0.,165.,0.)), np.array((255.,255.,255.)))
    mask= mask_1 | mask_2
    mask = mask_1 | mask_2
    kern_dilate = np.ones((8,8),np.uint8)
    kern_erode  = np.ones((3,3),np.uint8)
    mask= cv2.erode(mask,kern_erode)
    mask=cv2.dilate(mask,kern_dilate)
    return mask



def find_blob(blob):
    largest_contour=0
    cont_index=0
    _,contours,_= cv2.findContours(blob, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for idx, contour in enumerate(contours):
        area=cv2.contourArea(contour)
        if (area > largest_contour):
            largest_contour=area
            cont_index=idx
    x1 = 0
    y1 = 0
    radius = 0
    if len(contours) > 0:
        (x1,y1),radius  = cv2.minEnclosingCircle(contours[cont_index])
    return x1,  y1, radius



def receiveimage(data):
    #rospy.loginfo("CAM_DATA")
    global flag_imageshow, cvFrame
    bridge=CvBridge()
    cvFrame = bridge.imgmsg_to_cv2(data,"passthrough")


def color_det(event):
    global flag_imageshow, cvFrame, now, now_p, xt_image, yt_image, radius, detect
    frame = cvFrame
    now = timer()

    bridge=CvBridge()
    mask_red = segment_colour(frame)
    x_blob, y_blob, radius = find_blob(mask_red)
    if(radius<5):
        detect=0
        xt_image=-1
        yt_image=-1
        radius=-1
    else:
        detect = 1
        xt_image = x_blob
        yt_image = y_blob
        cv2.circle(frame, (int(xt_image),int(yt_image)), int(radius), (0, 255, 0), 2)
        cv2.circle(frame, (int(xt_image), int(yt_image)), 4, (110, 0, 255), -1)

    #re = cv2.resize(frame, (500, 500), interpolation = cv2.INTER_AREA)
    ReceiveTar()
    now_p = timer()
    #####      info.time = now     ### 
    
    if flag_imageshow == 1:
        cv2.imshow('detection',frame)
        cv2.waitKey(1)
    k = cv2.waitKey(5) & 0xff
    if k == 27:
        flag_imageshow = 0
        cv2.destroyAllWindows()
    #rate.sleep()



def ReceiveTar():
    global xt_image, yt_image, now, now_p, v_x, v_y, v_z, v_roll, v_pitch, v_yaw, x, y, z, roll, pitch, yaw, goal_pred, Rot_body_to_inertial, goal_pred_var, detect, v1, v2, x1_prev, x2_prev
    
    R = Rot_body_to_inertial
    vx = v_x
    vy = v_y
    xn = x
    yn = y
    zn = z
    #xt_image = data.center.x
    #yt_image = data.center.y
    #radius = data.radius
    #detect = data.detect
    #now = data.time

    if detect==0:
        rospy.loginfo(detect)
        #w1_prev = x1 + (float(X[1])-v_x)*del_t
        #w2_prev = x2 + (float(X[3])-v_y)*del_t
        #now_p = timer()
        pass
    else:
        #rospy.loginfo("FIL %s", fil1)
        del_t = now-now_p
        if del_t == 0:
            pass
        else:
            x1, x2 = get_position(xt_image, yt_image, xn, yn, R)    

            x1 = 0.65*x1 + 0.35*x1_prev  
            x2 = 0.65*x2 + 0.35*x2_prev   
            x1_prev = x1
            x2_prev = x2
            goal_pred = np.array([[x1]
                                ,[v1]
                                ,[x2]
                                ,[v2]])


            ##      initial drone ht 0.194387 changed now to 0 (IMPO)
            ##      rover height 0.43582
            ##      ==> landing height = 0.43583+0
            img = np.array([[x1-xn]
                        ,[x2-yn]
                        ,[0.43582]])

            goal_pred_var = np.array([[np.random.normal(0, 0.3*1.1**(float(img[0])*0.25/(z+0.0001))), 0, 0, 0]
                                    ,[0, np.random.normal(0, 1+8*(abs(v_pitch)+abs(v_roll))+0.5*abs(v_x*v_y)), 0, 0]
                                    ,[0, 0, np.random.normal(0, 0.3*1.1**(float(img[1])*0.25/(z+0.0001))), 0]
                                    ,[0, 0, 0, np.random.normal(0, 1+8*(abs(v_pitch)+abs(v_roll))+0.5*abs(v_x*v_y))]])   



def get_position(xt, yt, xn, yn, R):
    global hori_fov, vert_fov
    key_points_dir_body = np.array([[cos(np.pi/4-vert_fov)*cos(hori_fov), cos(np.pi/4-vert_fov)*cos(-hori_fov), cos(np.pi/4+vert_fov)*cos(hori_fov), cos(np.pi/4+vert_fov)*cos(-hori_fov), cos(np.pi/4)]
                                    ,[sin(hori_fov), sin(-hori_fov), sin(hori_fov), sin(-hori_fov), 0]
                                    ,[-sin(np.pi/4-vert_fov)*cos(hori_fov), -sin(np.pi/4-vert_fov)*cos(-hori_fov), -sin(np.pi/4+vert_fov)*cos(hori_fov), -sin(np.pi/4+vert_fov)*cos(-hori_fov), -sin(np.pi/4)]])
    key_points_dir_global = np.dot(R, key_points_dir_body)

    for i in range(len(key_points_dir_global[0])):
        key_points_dir_global[0][i] = float(key_points_dir_global[0][i])*(0.43582-z)/float(key_points_dir_global[2][i]) + xn
        key_points_dir_global[1][i] = float(key_points_dir_global[1][i])*(0.43582-z)/float(key_points_dir_global[2][i]) + yn
        key_points_dir_global[2][i] = 0.43582

    M1 = np.array([[float(key_points_dir_global[0][0]), float(key_points_dir_global[1][0]), 1, 0, 0, 0, 0, 0, 0]
                ,[0, 0, 0, float(key_points_dir_global[0][0]), float(key_points_dir_global[1][0]), 1, 0, 0, 0]
                ,[float(key_points_dir_global[0][1]), float(key_points_dir_global[1][1]), 1, 0, 0, 0, -500*float(key_points_dir_global[0][1]), -500*float(key_points_dir_global[1][1]), -500*1]
                ,[0, 0, 0, float(key_points_dir_global[0][1]), float(key_points_dir_global[1][1]), 1, 0, 0, 0]
                ,[float(key_points_dir_global[0][2]), float(key_points_dir_global[1][2]), 1, 0, 0, 0, 0, 0, 0]
                ,[0, 0, 0, float(key_points_dir_global[0][2]), float(key_points_dir_global[1][2]), 1, -500*float(key_points_dir_global[0][2]), -500*float(key_points_dir_global[1][2]), -500*1]
                ,[float(key_points_dir_global[0][3]), float(key_points_dir_global[1][3]), 1, 0, 0, 0, -500*float(key_points_dir_global[0][3]), -500*float(key_points_dir_global[1][3]), -500*1]
                ,[0, 0, 0, float(key_points_dir_global[0][3]), float(key_points_dir_global[1][3]), 1, -500*float(key_points_dir_global[0][3]), -500*float(key_points_dir_global[1][3]), -500*1]
                ,[float(key_points_dir_global[0][4]), float(key_points_dir_global[1][4]), 1, 0, 0, 0, -250*float(key_points_dir_global[0][4]), -250*float(key_points_dir_global[1][4]), -250*1]])

    M2 = np.array([[xt]
                ,[yt]
                ,[1]])
        
    U, D, V = np.linalg.svd(M1)
    M = np.reshape(V[len(V)-1], (3,3))
    M = np.linalg.inv(M)

    w1 = float(np.dot(M[0], M2)/np.dot(M[2], M2))
    w2 = float(np.dot(M[1], M2)/np.dot(M[2], M2))

    return w1, w2


def get_velocity(event2):
    global w1_prev, w2_prev, v1, v2, v1_prev, v2_prev, now, now_p

    dt = 0.5

    w1 = float(goal_pred[0])
    w2 = float(goal_pred[2])
    v1 = (w1-w1_prev)/dt
    v2 = (w2-w2_prev)/dt

    v1 = 0.6*v1+0.4*v1_prev
    v2 = 0.6*v2+0.4*v2_prev

    v1_prev = v1
    v2_prev = v2    

    w1_prev = w1
    w2_prev = w2



def callback(info):
    global x, y, z, roll, pitch, yaw, Rot_body_to_inertial, Rot_inertial_to_body, v_roll, v_pitch, v_yaw, v_x, v_y, v_z
    
    ############################        GAZEBO COORDINATE FRAME
    ###     Positions in global gazebo frame
    x = info.pose.pose.position.y
    y = -info.pose.pose.position.x
    z = info.pose.pose.position.z

    ###     All linear velocities are local 
    va = info.twist.twist.linear.x
    vb = info.twist.twist.linear.y
    vc = info.twist.twist.linear.z
    
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

    Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                                    ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                                    ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
    Rot_inertial_to_body = Rot_body_to_inertial.transpose()
    
    ###     All angular velocities are local
    v_roll = info.twist.twist.angular.x
    v_pitch = info.twist.twist.angular.y
    v_yaw = info.twist.twist.angular.z

    ###     Convert Velocities to the global frame

    v2 = np.array([[v_roll]
                ,[v_pitch]
                ,[v_yaw]])

    v1 = np.array([[va]
                ,[vb]
                ,[vc]])

    v2 = np.dot(Rot_body_to_inertial, v2)
    v1 = np.dot(Rot_body_to_inertial, v1)

    v_roll = float(v2[0])
    v_pitch = float(v2[1])
    v_yaw = float(v2[2])

    v_x = float(v1[0])
    v_y = float(v1[1])
    v_z = float(v1[2])



def kalman(event3):
    global X, H, P, v_x, v_y, v_z, x, y, z, goal_pred, goal_pred_var, detect
    del_t = 0.01
    if detect == 0:        
        Q = np.array([[np.random.normal(0, 1), 0, 0, 0]
                    ,[0, np.random.normal(0, 1), 0, 0]
                    ,[0, 0, np.random.normal(0, 1), 0]
                    ,[0, 0, 0, np.random.normal(0, 1)]])
    else:
        Q = np.array([[np.random.normal(0, 4), 0, 0, 0]
                    ,[0, np.random.normal(0, 1), 0, 0]
                    ,[0, 0, np.random.normal(0, 4), 0]
                    ,[0, 0, 0, np.random.normal(0, 1)]])
    v1 = float(X[1])
    v2 = float(X[3])
    A = np.array([[1, (del_t), 0, 0]
                ,[0, 1, 0, 0]
                ,[0, 0, 1, (del_t)]
                ,[0, 0, 0, 1]])

    X_new_pred = np.dot(A, X)
    
    P_k = np.dot(A, P)
    P_k = np.dot(P_k, A.transpose())
    Q = np.array([[np.random.normal(0, 3), 0, 0, 0]
                ,[0, np.random.normal(0, 1), 0, 0]
                ,[0, 0, np.random.normal(0, 3), 0]
                ,[0, 0, 0, np.random.normal(0, 1)]])
        
    P_k = P_k + Q
    
    mu_exp = np.dot(H, X_new_pred)
    std_dev_exp = np.dot(H.transpose(), P_k)
    std_dev_exp = np.dot(std_dev_exp, H)

    KG = np.dot(np.dot(std_dev_exp, H.transpose()), np.linalg.inv(std_dev_exp + goal_pred_var))

    X_new = X_new_pred + np.dot(KG, (np.dot(H,goal_pred) - np.dot(H,mu_exp)))
    
    X = X_new
    
    P = std_dev_exp - np.dot(KG, std_dev_exp)

    goal_find()
    #msg.goal.x = float(X[0])
    #msg.goal.y = float(X[2])
    #msg.goal.z = 0.43582
    #msg.vel.x = float(X[1])
    #msg.vel.y = float(X[3])
    #msg.vel.z = 0.0
    #msg.posn.x = x
    #msg.posn.y = y
    #msg.posn.z = z
    #msg.detected = detect
    #pub.publish(msg)


def goal_find():
    global goal, vel_rover, Rot_inertial_to_body, detect, X
    goal[0] = float(X[0])
    goal[1] = float(X[2])
    goal[2] = 0.43582 ##0.435
    v1 = float(X[1])
    v2 = float(X[3])
    v = np.array([[v1]
                ,[v2]
                ,[0.0]])
    rospy.loginfo("VEL %s", v)
    v = np.dot(Rot_inertial_to_body, v)
    vel_rover[0] = float(v[0])
    vel_rover[1] = float(v[1])
    vel_rover[2] = float(v[2])


def sdre(event1):
    global detect, x, y, z, roll, pitch, yaw, vel_rover, goal, goal_body, v_x, v_y, v_z, Rot_body_to_inertial, Rot_inertial_to_body, A_sdre
    #rospy.loginfo("GOAL_GLOBAL %s", goal) 
    goal_body[0] = goal[0] - x
    goal_body[1] = goal[1] - y
    goal_body[2] = goal[2] - z

    ####    Global to Body rotation
    goal_body = np.dot(Rot_inertial_to_body,goal_body.transpose())

    Q_sdre = np.array([[((5*goal_body[0])**2)/abs(goal_body[2]+0.0001)+1, 0, 0, 0, 0, 0]
                ,[0, abs(150*(0.5+abs(goal_body[2]))*(vel_rover[0]-v_x)/(0.001+0.1*abs(goal_body[0]))), 0, 0, 0, 0]
                ,[0, 0, ((5*goal_body[1])**2)/abs(goal_body[2]+0.0001)+1, 0, 0, 0]
                ,[0, 0, 0, abs(150*(0.5+abs(goal_body[2]))*(vel_rover[1]-v_y)/(0.001+0.1*abs(goal_body[1]))), 0, 0]
                ,[0, 0, 0, 0, 1+(10*goal_body[2]/sqrt(0.01+0.01*(goal_body[0]**2)+0.01*(goal_body[1]**2)))**2, 0]
                ,[0, 0, 0, 0, 0, 1/abs(goal_body[2]+0.001)]])

    R_sdre = np.array([[800, 0, 0]
                ,[0, 75000, 0]   #Pitch
                ,[0, 0, 75000]]) #Roll

    ###     Calculation for control done in body fixed frame
    X_sdre = np.array([[goal_body[0]],[vel_rover[0]-v_x],[goal_body[1]],[vel_rover[1]-v_y],[goal_body[2]],[vel_rover[2]-v_z]])

    ###     d2(e_x)/dt2 = 0-d2(x)/dt2 so all signs inverted
    B_sdre = np.array([[0, 0, 0], [0, -9.8, 0], [0, 0, 0], [0, 0, 9.8], [0, 0, 0], [-1, 0, 0]])

    P_sdre = la.solve_continuous_are(A_sdre, B_sdre, Q_sdre, R_sdre)

    u = np.dot(-np.linalg.inv(R_sdre),B_sdre.transpose())
    u = np.dot(u,P_sdre)
    u = np.dot(u,X_sdre)

    u0 = float(u[0])
    u1 = float(u[1])
    u2 = float(u[2])

    u0 = (u0*1.5 + 14.7)/29.4
    ##15 deg max cutoff at 10
    if u0>1:
        u0 = 1
    if u0<0:
        u0 = 0
    
    if Q_sdre[0][0]>Q_sdre[1][1]:
        if u1>10*np.pi/180:
            u1 = 10*np.pi/180
        if u1<-10*np.pi/180:
            u1 = -10*np.pi/180
    else:
        if u1>5*np.pi/180:
            u1 = 5*np.pi/180
        if u1<-5*np.pi/180:
            u1 = -5*np.pi/180

    if Q_sdre[2][2]>Q_sdre[3][3]:
        if u2>10*np.pi/180:
            u2 = 10*np.pi/180
        if u2<-10*np.pi/180:
            u2 = -10*np.pi/180
    else:
        if u2>5*np.pi/180:
            u2 = 5*np.pi/180
        if u2<-5*np.pi/180:
            u2 = -5*np.pi/180

    if sqrt(goal_body[0]**2+goal_body[1]**2)<0.8 and abs(goal_body[2])<1:
        rospy.loginfo("LAND")
        u0 = 0.0
        u1 = 0.0
        u2 = 0.0

    rospy.loginfo("Q %s",Q_sdre)

    quater = tf.transformations.quaternion_from_euler(u2,u1,yaw+np.pi/2) #0
    msg.header = Header()
    msg.type_mask = 0
    msg.orientation.x = quater[0]
    msg.orientation.y = quater[1]
    msg.orientation.z = quater[2]
    msg.orientation.w = quater[3]
    msg.body_rate.x = 0.0
    msg.body_rate.y = 0.0
    msg.body_rate.z = 0.0
    msg.thrust = u0

    pub.publish(msg)


def listener():
    rospy.Subscriber('/camera_on_quad/images', Image, receiveimage)
    timer=rospy.Timer(rospy.Duration(20/1000.0),color_det)
    rospy.Subscriber("/drone/mavros/local_position/odom", Odometry, callback)
    #timer1=rospy.Timer(rospy.Duration(20/1000.0),ReceiveTar)
    timer1=rospy.Timer(rospy.Duration(12/1000.0),sdre)
    timer2=rospy.Timer(rospy.Duration(500/1000.0),get_velocity)
    timer3=rospy.Timer(rospy.Duration(10/1000.0),kalman)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass