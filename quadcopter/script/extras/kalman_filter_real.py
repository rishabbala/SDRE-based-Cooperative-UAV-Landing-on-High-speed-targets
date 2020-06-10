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
from test.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from quadcopter.msg import *
import time
import control.matlab as mb
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
x = 0.0
y = 0.0
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
hori_fov = np.pi/4              #on either side
vert_fov = 720*hori_fov/1280    #compute vert_fov from hori_fov

H = np.array([[1, 0, 0, 0]
            ,[0, 1, 0, 0]
            ,[0, 0, 1, 0]
            ,[0, 0, 0, 1]])

goal_pred = np.array([[0.0]
                    ,[0.0]
                    ,[0.0]
                    ,[0.0]])    #target states estimated by the camera homography

goal_pred_var = np.array([[1, 0, 0, 0]
                        ,[0, 1, 0, 0]
                        ,[0, 0, 1, 0]
                        ,[0, 0, 0, 1]])     #variance in the estimates by the camera homography

##      Rotational matrices
Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                                ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                                ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
Rot_inertial_to_body = Rot_body_to_inertial.transpose()


X = np.array([[0.0]
            ,[0.0]
            ,[0.0]
            ,[0.0]])        #target states estimated by the kalman filter

P = np.array([[np.random.normal(0, 1), 0, 0, 0]
            ,[0, np.random.normal(0, 0.25), 0, 0]
            ,[0, 0, np.random.normal(0, 1), 0]
            ,[0, 0, 0, np.random.normal(0, 0.25)]])     #variance in the filter

msg = kalman()

def kalman(timer):
    global now_kal, now_kal_p, X, P, v_x, v_y, v_z, x, y, z, goal_pred, goal_pred_var, detect
    del_t = 0.01
    if detect == 0:
        ####    If not detected assume no noise in prediction and inf noise in measurement
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

    #prediction of the next state
    X_new_pred = np.dot(A, X)

    #variance in the prediction
    P_k = np.dot(A, P)
    P_k = np.dot(P_k, A.transpose())
    Q = np.array([[np.random.normal(0, 3), 0, 0, 0]
                ,[0, np.random.normal(0, 1), 0, 0]
                ,[0, 0, np.random.normal(0, 3), 0]
                ,[0, 0, 0, np.random.normal(0, 1)]])
        
    P_k = P_k + Q
    

    #mu_exp is the prediction, without the camera data. As the prediction states and the kalman filter states are the same, H = Identity Matrix
    #So mu_exp and std_dev_exp are the same as P_k and X_new_pred
    mu_exp = np.dot(H, X_new_pred)
    std_dev_exp = np.dot(H.transpose(), P_k)
    std_dev_exp = np.dot(std_dev_exp, H)

    #Compute the Kalman Gain, using the prediction from the previous state and measurement from camera
    KG = np.dot(np.dot(std_dev_exp, H.transpose()), np.linalg.inv(std_dev_exp + goal_pred_var))

    #Compute the new states
    X_new = X_new_pred + np.dot(KG, (np.dot(H,goal_pred) - np.dot(H,mu_exp)))
    X = X_new
    
    #Compute the new variance in states
    P = std_dev_exp - np.dot(KG, std_dev_exp)

    #Send the dara
    msg.goal.x = float(X[0])-x
    msg.goal.y = float(X[2])-y
    msg.goal.z = 0.0
    msg.vel.x = float(X[1])
    msg.vel.y = float(X[3])
    msg.vel.z = 0.0
    msg.posn.x = x
    msg.posn.y = y
    msg.posn.z = z
    msg.detected = detect
    pub.publish(msg)


def ReceiveTar(data):
    global i, now_cam, now_cam_p, v_x, v_y, v_z, v_roll, v_pitch, v_yaw, x, y, z, roll, pitch, yaw, goal_pred, Rot_body_to_inertial, goal_pred_var, detect, v1, v2, x1_prev, x2_prev
    
    R = Rot_body_to_inertial
    vx = v_x
    vy = v_y
    xn = x
    yn = y
    zn = z
    xt_image = data.center.x
    yt_image = data.center.y
    radius = data.radius
    detect = data.detected
    now_cam = data.time

    if detect==0:
        rospy.loginfo(detect)
        pass
    else:
        #Get time
        del_t = now_cam-now_cam_p
        if del_t == 0:
            pass
        else:
            #Get the estimated positions from the camera measurement only
            x1, x2 = get_position(xt_image, yt_image, xn, yn, R)    

            #Average it to smoothen the data
            x1 = 0.65*x1 + 0.35*x1_prev  
            x2 = 0.65*x2 + 0.35*x2_prev   
            x1_prev = x1
            x2_prev = x2
 
            #Set these new values as the new predicted values from the camera measurement
            #Velocity is estimated by get_velociy fn which is called in const time intervals at the end
            goal_pred = np.array([[x1]
                                ,[v1]
                                ,[x2]
                                ,[v2]])

            img = np.array([[x1-xn]
                        ,[x2-yn]
                        ,[0.0]])


            #Set the variance in the measurements
            goal_pred_var = np.array([[np.random.normal(0, 0.3*1.1**(float(img[0])*0.25/(z+0.0001))), 0, 0, 0]
                                    ,[0, np.random.normal(0, 1+12*(abs(v_pitch)+abs(v_roll))+0.5*abs(v_x*v_y)+1/(0.25+abs(z-0.0))), 0, 0]
                                    ,[0, 0, np.random.normal(0, 0.3*1.1**(float(img[1])*0.25/(z+0.0001))), 0]
                                    ,[0, 0, 0, np.random.normal(0, 1+12*(abs(v_pitch)+abs(v_roll))+0.5*abs(v_x*v_y)+1/(0.25+abs(z-0.0)))]])

            now_cam_p = data.time
            i+=1
        
def get_position(xt, yt, xn, yn, R):
    global vert_fov, hori_fov

    #Find directional vectors corresponding to (0,0), (w,0), (0,h), (w,h), (w/2,h/2) in the image plane using the fov of camera
    #Camera oriented at 45 deg
    key_points_dir_body = np.array([[cos(np.pi/4-vert_fov)*cos(hori_fov), cos(np.pi/4-vert_fov)*cos(-hori_fov), cos(np.pi/4+vert_fov)*cos(hori_fov), cos(np.pi/4+vert_fov)*cos(-hori_fov), cos(np.pi/4)]
                                    ,[sin(hori_fov), sin(-hori_fov), sin(hori_fov), sin(-hori_fov), 0]
                                    ,[-sin(np.pi/4-vert_fov)*cos(hori_fov), -sin(np.pi/4-vert_fov)*cos(-hori_fov), -sin(np.pi/4+vert_fov)*cos(hori_fov), -sin(np.pi/4+vert_fov)*cos(-hori_fov), -sin(np.pi/4)]])
    
    #Convert these vectors to the global frame
    key_points_dir_global = np.dot(R, key_points_dir_body)

    #Find the (x,y) coordinate where they intersect the plane of z=0.0, which is the height of the rover
    for i in range(len(key_points_dir_global[0])):
        t1 = float(key_points_dir_global[0][i])*(0.0-z)/float(key_points_dir_global[2][i])
        t2 = float(key_points_dir_global[1][i])*(0.0-z)/float(key_points_dir_global[2][i])

        key_points_dir_global[0][i] = t1*cos(yaw) - t2*sin(yaw) + xn
        key_points_dir_global[1][i] = t1*sin(yaw) + t2*cos(yaw) + yn
        key_points_dir_global[2][i] = 0.0

    #Compute the matrices M1 and M2 for computing the Homography Matrix M using SVD
    #refer https://classroom.udacity.com/courses/ud955/lessons/3839218552/concepts/30084186810923
    M1 = np.array([[float(key_points_dir_global[0][0]), float(key_points_dir_global[1][0]), 1, 0, 0, 0, 0, 0, 0]
                ,[0, 0, 0, float(key_points_dir_global[0][0]), float(key_points_dir_global[1][0]), 1, 0, 0, 0]
                ,[float(key_points_dir_global[0][1]), float(key_points_dir_global[1][1]), 1, 0, 0, 0, -1280*float(key_points_dir_global[0][1]), -1280*float(key_points_dir_global[1][1]), -1280*1]
                ,[0, 0, 0, float(key_points_dir_global[0][1]), float(key_points_dir_global[1][1]), 1, 0, 0, 0]
                ,[float(key_points_dir_global[0][2]), float(key_points_dir_global[1][2]), 1, 0, 0, 0, 0, 0, 0]
                ,[0, 0, 0, float(key_points_dir_global[0][2]), float(key_points_dir_global[1][2]), 1, -720*float(key_points_dir_global[0][2]), -720*float(key_points_dir_global[1][2]), -720*1]
                ,[float(key_points_dir_global[0][3]), float(key_points_dir_global[1][3]), 1, 0, 0, 0, -1280*float(key_points_dir_global[0][3]), -1280*float(key_points_dir_global[1][3]), -1280*1]
                ,[0, 0, 0, float(key_points_dir_global[0][3]), float(key_points_dir_global[1][3]), 1, -720*float(key_points_dir_global[0][3]), -720*float(key_points_dir_global[1][3]), -720*1]
                ,[float(key_points_dir_global[0][4]), float(key_points_dir_global[1][4]), 1, 0, 0, 0, -640*float(key_points_dir_global[0][4]), -640*float(key_points_dir_global[1][4]), -640*1]])

    M2 = np.array([[xt]
                ,[yt]
                ,[1]])
        
    U, D, V = np.linalg.svd(M1)
    M = np.reshape(V[len(V)-1], (3,3))
    M = np.linalg.inv(M)

    #compute the positions
    w1 = float(np.dot(M[0], M2)/np.dot(M[2], M2))
    w2 = float(np.dot(M[1], M2)/np.dot(M[2], M2))

    return w1, w2

def get_velocity(event):
    global u1_prev, u2_prev, u3_prev, v1, v2, v1_prev, v2_prev

    #The function is called at const intervals
    dt = 0.5

    #Get the current positions
    w1 = float(goal_pred[0])
    w2 = float(goal_pred[2])

    #Calculate the velocities
    v1_n = (w1-u1_prev)/dt
    v2_n = (w2-u2_prev)/dt

    #Average the estimated velocity to smoothen it
    v1 = 0.6*v1_n+0.4*v1_prev    
    v2 = 0.6*v2_n+0.4*v2_prev

    #Update the previous velocity values
    v1_prev = v1
    v2_prev = v2    

    u1_prev = w1
    u2_prev = w2


def callback(info):
    global x, y, z, roll, pitch, yaw, Rot_body_to_inertial, Rot_inertial_to_body, v_roll, v_pitch, v_yaw, v_x, v_y, v_z

    #Get the data from mavros
    #!!!!!!!!!!!        MAKE SURE TO CHECK THE AXES AS THEY ARE ALL INVERTED IN ALL OF MY CODES      !!!!!!!!!!!!!!!!
    x = info.pose.pose.position.x
    y = info.pose.pose.position.y
    z = info.pose.pose.position.z

    va = info.twist.twist.linear.x
    vb = info.twist.twist.linear.y
    vc = info.twist.twist.linear.z
    
    a1 = info.pose.pose.orientation.x
    b1 = info.pose.pose.orientation.y
    c1 = info.pose.pose.orientation.z
    d1 = info.pose.pose.orientation.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])

    #yaw = yaw-np.pi/2
    #if yaw<np.pi/2:
    #    yaw = yaw+2*np.pi/2
    #if yaw>np.pi/2:
    #    yaw = yaw-2*np.pi/2

    Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                                    ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                                    ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
    Rot_inertial_to_body = Rot_body_to_inertial.transpose()
    
    v_roll = info.twist.twist.angular.x
    v_pitch = info.twist.twist.angular.y
    v_yaw = info.twist.twist.angular.z

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
                
def listener():
    rospy.Subscriber('/landing_target_info_new', TargetInfo,ReceiveTar)
    rospy.Subscriber("/drone/mavros/local_position/odom", Odometry, callback)
    timer=rospy.Timer(rospy.Duration(10/1000.0),kalman)
    timer2=rospy.Timer(rospy.Duration(500/1000.0),get_velocity)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass