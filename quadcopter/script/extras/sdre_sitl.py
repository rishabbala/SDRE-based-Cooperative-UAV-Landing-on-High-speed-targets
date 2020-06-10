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
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from quadcopter.msg import *
import time
import control.matlab as mb
import csv
from timeit import default_timer as timer

rospy.init_node('sdre', anonymous=True)
pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

#posn = open('posn4.csv', 'w')

roll = 0.0
pitch = 0.0
yaw = 0.0
detect = 1
now = timer()

msg = AttitudeTarget()

###     Goal in gazebo frame with origin as start, from the start point of drone
goal = np.array([0.0, 0.0, 0.0])

goal_body = np.array([0.0, 0.0, 0.0])

x = 0.0
y = 0.0
z = 0.0
x_r = 0.0
y_r = 0.0
v_x = 0.0
v_y = 0.0
v_z = 0.0

vel_rover = [0,0,0]

A = np.array([[0, 1, 0, 0, 0, 0]
            ,[0, 0, 0, 0, 0, 0]
            ,[0, 0, 0, 1, 0, 0]
            ,[0, 0, 0, 0, 0, 0]
            ,[0, 0, 0, 0, 0, 1]
            ,[0, 0, 0, 0, 0, 0]])

Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                        ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                        ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
Rot_inertial_to_body = Rot_body_to_inertial.transpose()


def sdre():
    while not rospy.is_shutdown():
        now = timer()
        goal[0] = 8
        goal[1] = 15
        goal[2] = 0
        detect = 1

        ##      Receive vel info and convert to body fixed frame
        v1 = 0.0
        v2 = 0.0
        v = np.array([[v1]
                    ,[v2]
                    ,[0.0]])
        #print("Roll_in = ", roll)
        v = np.dot(Rot_inertial_to_body, v)
        vel_rover[0] = 0.0
        vel_rover[1] = 0.0
        vel_rover[2] = 0.0
        global x_r, y_r, detect, x, y, z, roll, pitch, yaw, vel_rover, goal, goal_body, v_x, v_y, v_z, Rot_body_to_inertial, Rot_inertial_to_body
        ####    Global to Body conversion for the goal

        #posn.write('%f;' % float(x))
        #posn.write('%f;' % float(z))
        #posn.write('%f;' % float(x_r))
        #posn.write('%f;' % float(now))
        #posn.write('\n')

        goal_body[0] = goal[0] - x
        goal_body[1] = goal[1] - y
        goal_body[2] = goal[2] - z

        goal_body = np.dot(Rot_inertial_to_body,goal_body.transpose())

        ####    Weighting Matrices Q R
        Q = np.array([[((5*goal_body[0])**2)/abs(goal_body[2]+0.0001)+1, 0, 0, 0, 0, 0]
                    ,[0, abs(150*(0.5+abs(goal_body[2]))*(vel_rover[0]-v_x)/(0.001+0.1*abs(goal_body[0]))), 0, 0, 0, 0]
                    ,[0, 0, ((5*goal_body[1])**2)/abs(goal_body[2]+0.0001)+1, 0, 0, 0]
                    ,[0, 0, 0, abs(150*(0.5+abs(goal_body[2]))*(vel_rover[1]-v_y)/(0.001+0.1*abs(goal_body[1]))), 0, 0]
                    ,[0, 0, 0, 0, 1+(10*goal_body[2]/sqrt(0.01+0.01*(goal_body[0]**2)+0.01*(goal_body[1]**2)))**2, 0]
                    ,[0, 0, 0, 0, 0, 1/abs(goal_body[2]+0.001)]])

        R = np.array([[800, 0, 0]    #z - accn
                    ,[0, 75000, 0]   #Pitch
                    ,[0, 0, 75000]]) #Roll

        ####    Calculation for control done in body fixed frame
        ###     d2(e_x)/dt2 = 0-d2(x)/dt2 so all signs inverted
        X = np.array([[goal_body[0]],[vel_rover[0]-v_x],[goal_body[1]],[vel_rover[1]-v_y],[goal_body[2]],[vel_rover[2]-v_z]])


        B = np.array([[0, 0, 0], [0, -9.8, 0], [0, 0, 0], [0, 0, 9.8], [0, 0, 0], [-1, 0, 0]])

        P = la.solve_continuous_are(A, B, Q, R)

        u = np.dot(-np.linalg.inv(R),B.transpose())
        u = np.dot(u,P)
        u = np.dot(u,X)

        u0 = float(u[0])
        u1 = float(u[1])
        u2 = float(u[2])

        ####    Normalizing the received thrust
        u0 = (u0*1.5 + 14.7)/29.4

        ####    Restrict rotation angles to 10 deg
        if u0>1:
            u0 = 1
        if u0<0:
            u0 = 0

        if u1>10*np.pi/180:
            u1 = 10*np.pi/180
        if u1<-10*np.pi/180:
            u1 = -10*np.pi/180

        if u2>10*np.pi/180:
            u2 = 10*np.pi/180
        if u2<-10*np.pi/180:
            u2 = -10*np.pi/180
        

        ####    Start descending for small errors
        if sqrt(goal_body[0]**2+goal_body[1]**2)<0.8 and abs(goal_body[2])<1:
            rospy.loginfo("LAND")
            u0 = 0.0
            u1 = 0.0
            u2 = 0.0


        ####    Convert to quaternions and publish
        quater = tf.transformations.quaternion_from_euler(u2,u1,yaw) #0
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


        #print(X)
        #print("u2roll = ", u2)
        #print("Pitch = ", u1,pitch)
        rate = rospy.Rate(100) 
        rate.sleep


def callback(info):
    ##MUST GET HEADING
    global x, y, z, roll, pitch, yaw, vel_rover, vel_drone_rot, vel_drone_trans, head, error_head_prev, goal, goal_body, v_x, v_y, v_z, Rot_body_to_inertial, Rot_inertial_to_body
    
    ###     Positions in global gazebo frame
    x = info.pose.pose.position.x
    y = info.pose.pose.position.y
    z = info.pose.pose.position.z

    ###     All linear velocities are local 
    v_x = info.twist.twist.linear.x
    v_y = info.twist.twist.linear.y
    v_z = info.twist.twist.linear.z
    
    ###     Orientations in order of rotation
    a1 = info.pose.pose.orientation.x
    b1 = info.pose.pose.orientation.y
    c1 = info.pose.pose.orientation.z
    d1 = info.pose.pose.orientation.w

    roll, pitch, yaw = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])
    #print("Roll_call = ", roll)

    ###     Yaw in gazebo frame
    #yaw = yaw-np.pi/2
    #if yaw<np.pi/2:
    #    yaw = yaw+2*np.pi/2
    #if yaw>np.pi/2:
    #    yaw = yaw-2*np.pi/2

    Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                                    ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                                    ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
    Rot_inertial_to_body = Rot_body_to_inertial.transpose()

###     Add integral error
###     No land if no detect

'''
def ReceiveTar(info):
    global goal, vel_rover, Rot_inertial_to_body, detect, x, y, v_x, v_y

    ##      Receive position info
    goal[0] = 8
    goal[1] = 0
    goal[2] = 0
    detect = 1

    ##      Receive vel info and convert to body fixed frame
    v1 = 0.0
    v2 = 0.0
    v = np.array([[v1]
                ,[v2]
                ,[0.0]])
    v = np.dot(Rot_inertial_to_body, v)
    vel_rover[0] = float(v[0])
    vel_rover[1] = float(v[1])
    vel_rover[2] = float(v[2])
         
'''       

#def callback2(info):
#    global x_r, y_r
#    x_r = info.pose.pose.position.y
#    y_r = -info.pose.pose.position.x

def listener():
    #rospy.Subscriber("/rover/mavros/local_position/odom", Odometry, callback2)
    rospy.Subscriber("/mavros/local_position/odom", Odometry, callback)
    #rospy.Subscriber('/kalman_filter', kalman, ReceiveTar)
    sdre()
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        posn.close()
        pass