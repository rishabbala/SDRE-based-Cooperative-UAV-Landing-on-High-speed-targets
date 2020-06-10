#!/usr/bin/env python
import rospy
import tf
import scipy.linalg as la
import scipy.signal as sig
import numpy as np
from math import *
import mavros_msgs.srv
from mavros_msgs.msg import AttitudeTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import *
# from test.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from quadcopter.msg import *
import time
# import control.matlab as mb
from timeit import default_timer as timer
from quadcopter.msg import *
import timeit

rospy.init_node('kalman_filter', anonymous=True)
pub = rospy.Publisher("/kalman_filter", kalman, queue_size=10)

roll = 0.0
pitch = 0.0
yaw = 0.0
v_x = 0.0
v_y = 0.0
v_z = 0.0
z = 0.0
u1_prev = 0.0
u2_prev = 0.0
u3_prev = 0.0
x1_prev = 0.0
x2_prev = 0.0
v_roll = 0.0
v_pitch = 0.0
v_yaw = 0.0
v1_prev = 0.0
v2_prev = 0.0
i = 0
v1 = 0.0
v2 = 0.0

detect = 1

now_cam_p = timer()
now_cam = timer()
now_kal_p = timer()
now_kal = timer()

####    Vert and hor fov
hori_fov = np.pi/4 #on either side
vert_fov = 2000*hori_fov/2000

H = np.array([[1, 0, 0, 0]
            ,[0, 1, 0, 0]
            ,[0, 0, 1, 0]
            ,[0, 0, 0, 1]])

goal_pred = np.array([[0.0]
                    ,[0.0]
                    ,[0.0]
                    ,[0.0]])

goal_pred_var = np.array([[1, 0, 0, 0]
                        ,[0, 1, 0, 0]
                        ,[0, 0, 1, 0]
                        ,[0, 0, 0, 1]])

Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                                ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                                ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
Rot_inertial_to_body = Rot_body_to_inertial.transpose()


X = np.array([[0.0]
            ,[0.0]
            ,[0.0]
            ,[0.0]])

P = np.array([[np.random.normal(0, 1), 0, 0, 0]
            ,[0, np.random.normal(0, 0.25), 0, 0]
            ,[0, 0, np.random.normal(0, 1), 0]
            ,[0, 0, 0, np.random.normal(0, 0.25)]])

msg = kalman()

def kalman(timer):
    global now_kal, now_kal_p, X, P, v_x, v_y, v_z, z, goal_pred, goal_pred_var, detect
    del_t = 0.01
    if detect == 0:
        ####    If not detected assume no noise in kalman filter and inf noise in measurement
        Q = np.array([[np.random.normal(0, 1), 0, 0, 0]
                    ,[0, np.random.normal(0, 1), 0, 0]
                    ,[0, 0, np.random.normal(0, 1), 0]
                    ,[0, 0, 0, np.random.normal(0, 1)]])
    else:
        Q = np.array([[np.random.normal(0, 4), 0, 0, 0]
                    ,[0, np.random.normal(0, 1), 0, 0]
                    ,[0, 0, np.random.normal(0, 4), 0]
                    ,[0, 0, 0, np.random.normal(0, 1)]])

    ####    Apply kalman filter
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
    msg.goal.x = float(X[0])
    msg.goal.y = float(X[2])
    msg.goal.z = 0.43582
    msg.vel.x = float(X[1])
    msg.vel.y = float(X[3])
    msg.vel.z = 0.0
    msg.posn.x = 0.0
    msg.posn.y = 0.0
    msg.posn.z = 0.435
    msg.detected = detect

    print(msg)
    pub.publish(msg)


def ReceiveTar(data):
    global i, now_cam, now_cam_p, v_x, v_y, v_z, v_roll, v_pitch, v_yaw, z, roll, pitch, yaw, goal_pred, Rot_body_to_inertial, goal_pred_var, detect, v1, v2, x1_prev, x2_prev

    R = Rot_body_to_inertial
    vx = v_x
    vy = v_y
    xt_image = data.center.x
    yt_image = data.center.y
    radius = data.radius
    detect = data.detected
    now_cam = data.time

    if detect==0:
        rospy.loginfo(detect)
        pass
    else:
        del_t = now_cam-now_cam_p
        if del_t == 0:
            pass
        else:
            x1, x2 = get_position(xt_image, yt_image, R)

            x1 = 0.65*x1 + 0.35*x1_prev
            x2 = 0.65*x2 + 0.35*x2_prev
            x1_prev = x1
            x2_prev = x2

            goal_pred = np.array([[x1]
                                ,[v1]
                                ,[x2]
                                ,[v2]])

            img = np.array([[x1]
                        ,[x2]
                        ,[0.43582]])

            goal_pred_var = np.array([[np.random.normal(0, 0.3*1.1**(float(img[0])*0.25/(z+0.0001))), 0, 0, 0]
                                    ,[0, np.random.normal(0, 1+12*(abs(v_pitch)+abs(v_roll))+0.5*abs(v_x*v_y)+1/(0.25+abs(z-0.43582))), 0, 0]
                                    ,[0, 0, np.random.normal(0, 0.3*1.1**(float(img[1])*0.25/(z+0.0001))), 0]
                                    ,[0, 0, 0, np.random.normal(0, 1+12*(abs(v_pitch)+abs(v_roll))+0.5*abs(v_x*v_y)+1/(0.25+abs(z-0.43582)))]])

            now_cam_p = data.time
            i+=1

def get_position(xt, yt, R):
    global vert_fov, hori_fov, z
    key_points_dir_body = np.array([[cos(np.pi/4-vert_fov)*cos(hori_fov), cos(np.pi/4-vert_fov)*cos(-hori_fov), cos(np.pi/4+vert_fov)*cos(hori_fov), cos(np.pi/4+vert_fov)*cos(-hori_fov), cos(np.pi/4)]
                                    ,[sin(hori_fov), sin(-hori_fov), sin(hori_fov), sin(-hori_fov), 0]
                                    ,[-sin(np.pi/4-vert_fov)*cos(hori_fov), -sin(np.pi/4-vert_fov)*cos(-hori_fov), -sin(np.pi/4+vert_fov)*cos(hori_fov), -sin(np.pi/4+vert_fov)*cos(-hori_fov), -sin(np.pi/4)]])
    key_points_dir_global = np.dot(R, key_points_dir_body)

    for i in range(len(key_points_dir_global[0])):
        key_points_dir_global[0][i] = float(key_points_dir_global[0][i])*(0.43582-z)/float(key_points_dir_global[2][i])
        key_points_dir_global[1][i] = float(key_points_dir_global[1][i])*(0.43582-z)/float(key_points_dir_global[2][i])
        key_points_dir_global[2][i] = 0.43582

    M1 = np.array([[float(key_points_dir_global[0][0]), float(key_points_dir_global[1][0]), 1, 0, 0, 0, 0, 0, 0]
                ,[0, 0, 0, float(key_points_dir_global[0][0]), float(key_points_dir_global[1][0]), 1, 0, 0, 0]
                ,[float(key_points_dir_global[0][1]), float(key_points_dir_global[1][1]), 1, 0, 0, 0, -2000*float(key_points_dir_global[0][1]), -2000*float(key_points_dir_global[1][1]), -2000*1]
                ,[0, 0, 0, float(key_points_dir_global[0][1]), float(key_points_dir_global[1][1]), 1, 0, 0, 0]
                ,[float(key_points_dir_global[0][2]), float(key_points_dir_global[1][2]), 1, 0, 0, 0, 0, 0, 0]
                ,[0, 0, 0, float(key_points_dir_global[0][2]), float(key_points_dir_global[1][2]), 1, -2000*float(key_points_dir_global[0][2]), -2000*float(key_points_dir_global[1][2]), -2000*1]
                ,[float(key_points_dir_global[0][3]), float(key_points_dir_global[1][3]), 1, 0, 0, 0, -2000*float(key_points_dir_global[0][3]), -2000*float(key_points_dir_global[1][3]), -2000*1]
                ,[0, 0, 0, float(key_points_dir_global[0][3]), float(key_points_dir_global[1][3]), 1, -2000*float(key_points_dir_global[0][3]), -2000*float(key_points_dir_global[1][3]), -2000*1]
                ,[float(key_points_dir_global[0][4]), float(key_points_dir_global[1][4]), 1, 0, 0, 0, -1000*float(key_points_dir_global[0][4]), -1000*float(key_points_dir_global[1][4]), -1000*1]])


    M2 = np.array([[xt]
                ,[yt]
                ,[1]])

    U, D, V = np.linalg.svd(M1)
    M = np.reshape(V[len(V)-1], (3,3))
    M = np.linalg.inv(M)

    w1 = float(np.dot(M[0], M2)/np.dot(M[2], M2))
    w2 = float(np.dot(M[1], M2)/np.dot(M[2], M2))

    return w1, w2

def get_velocity(event):
    global u1_prev, u2_prev, u3_prev, v1, v2, v1_prev, v2_prev

    dt = 0.5

    w1 = float(goal_pred[0])
    w2 = float(goal_pred[2])
    v1_n = (w1-u1_prev)/dt
    v2_n = (w2-u2_prev)/dt

    v1 = 0.6*v1_n+0.4*v1_prev

    v2 = 0.6*v2_n+0.4*v2_prev

    v1_prev = v1
    v2_prev = v2

    u1_prev = w1
    u2_prev = w2


def callback(info):
    global z, roll, pitch, yaw, Rot_body_to_inertial, Rot_inertial_to_body, v_roll, v_pitch, v_yaw, v_x, v_y, v_z

    global z, roll, pitch, yaw, Rot_body_to_inertial, Rot_inertial_to_body, v_roll, v_pitch, v_yaw, v_x, v_y, v_z

    z = info.pose.pose.position.z

    v_x = info.twist.twist.linear.x
    v_y = info.twist.twist.linear.y
    v_z = info.twist.twist.linear.z

    a1 = info.pose.pose.orientation.x
    b1 = info.pose.pose.orientation.y
    c1 = info.pose.pose.orientation.z
    d1 = info.pose.pose.orientation.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])

    yaw = yaw-np.pi/2

    Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                                    ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                                    ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
    Rot_inertial_to_body = Rot_body_to_inertial.transpose()

    v_roll = info.twist.twist.angular.x
    v_pitch = info.twist.twist.angular.y
    v_yaw = info.twist.twist.angular.z


def listener():
    rospy.Subscriber('/landing_target_info_new', TargetInfo,ReceiveTar)
    rospy.Subscriber("/drone0/mavros/local_position/odom", Odometry, callback)
    timer=rospy.Timer(rospy.Duration(10/1000.0),kalman)
    timer2=rospy.Timer(rospy.Duration(500/1000.0),get_velocity)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
