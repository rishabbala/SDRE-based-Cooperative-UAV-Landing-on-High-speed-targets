#!/usr/bin/env python
from __future__ import division
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
import csv
from timeit import default_timer as timer
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from quadcopter.msg import land
import mavros
from mavros import *

# #########################################################################################################################################################
#
# This is the code for landing multiple UAVs on a moving target with/without coordination
# Must be used along with rover_traj.py which generates rover path and relevant flags
# We use two drones, first with vision, second without vision
# Easily extended to more drones by changing in two_bot.launch, world file and topics needed here
#
# #########################################################################################################################################################

## Input which UAV is being controlled
bot_no = input("Enter bot number: ")
rospy.init_node('sdre'+str(bot_no), anonymous=True)
pub = rospy.Publisher("/drone"+str(bot_no)+"/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)

## Getting plots
# posn = open('oscillate_multi_coord.csv', 'w')

## Drone orientations
roll = 0.0
pitch = 0.0
yaw = 0.0
## Rover orientation
yaw2 = 0.0
detect = 0
now = rospy.get_time()
now_p = rospy.get_time()

msg = AttitudeTarget()

## Goal is position of rover, goal_body finally contains errors in B
goal = np.array([0.0, 0.0, 0.0])
goal_body = np.array([0.0, 0.0, 0.0])

## Information of drone
x = 0.0
y = 0.0
z = 0.0
v_x = 0.0
v_y = 0.0
v_z = 0.0

## Rover prev velocities for acc calculations
v1_p = 0.0
v2_p = 0.0
v3 = 0.0 ## Angular velocity about Z

acc_rover = [0,0,0]
vel_rover = [0,0,0]
pos_rover = [0,0,0]

A = np.array([[0, 1, 0, 0, 0, 0, 0]
            ,[0, 0, 0, 0, 0, 0, 0]
            ,[0, 0, 0, 1, 0, 0, 0]
            ,[0, 0, 0, 0, 0, 0, 0]
            ,[0, 0, 0, 0, 0, 1, 0]
            ,[0, 0, 0, 0, 0, 0, 0]
            ,[0, 0, 0, 0, 0, 0, 0]])

Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                        ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                        ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
Rot_inertial_to_body = Rot_body_to_inertial.transpose()

change_landing = 0
land_flag = []
zhold = 10 # initial height, can also be taken as input
def sdre():
    while not rospy.is_shutdown():
        global x, y, z, roll, pitch, yaw, vel_rover, goal, goal_body, v_x, v_y, v_z, Rot_body_to_inertial, Rot_inertial_to_body, yaw2, acc_rover, v3, zhold, change_landing, land_flag, bot_no, pos_rover

        # posn.write('%f;' % float(x))
        # posn.write('%f;' % float(y))
        # posn.write('%f;' % float(z))
        # posn.write('%f;' % float(v_x))
        # posn.write('%f;' % float(v_y))
        # posn.write('%f;' % float(v_z))
        # posn.write('%f;' % float(pos_rover[0]))
        # posn.write('%f;' % float(pos_rover[1]))
        # posn.write('%f;' % float(pos_rover[2]))
        # posn.write('%f;' % float(vel_rover[0]))
        # posn.write('%f;' % float(vel_rover[1]))
        # posn.write('%f;' % float(vel_rover[2]))
        # posn.write('\n')

        ## If height > 2 and not vision based landing, follow form behind the rover, dont go too close
        if z>4 and bot_no!=0:
            goal[0] = goal[0] - 2*cos(yaw2)*(1/(1+exp(-(z-6))))
            goal[1] = goal[1] - 2*sin(yaw2)*(1/(1+exp(-(z-6))))

        if land_flag!=[]:
            if land_flag[bot_no] == 0 and change_landing!=100:
                goal[2] = zhold

            if land_flag[bot_no] == 10 and bot_no!=0:
                goal[2] = land_flag[2]
            if land_flag[bot_no] == 10 and bot_no==0:
                goal[2] = land_flag[2]-z

            if bot_no == 0 and change_landing!=100:
                goal_body[0] = goal[0]
                goal_body[1] = goal[1]
                goal_body[2] = goal[2]
            else:
                goal_body[0] = goal[0] - x
                goal_body[1] = goal[1] - y
                goal_body[2] = goal[2] - z

            goal_body = np.dot(Rot_inertial_to_body,goal_body.transpose())

            # If mislanded, especially vision based, start regaining altitude
            if bot_no!=0 and ((sqrt(goal_body[0]**2 + goal_body[1]**2)>1 and change_landing!=100 and land_flag[bot_no]==1) or (detect==0 and bot_no==0 and land_flag[bot_no]==1)):
                goal[2] = sqrt(goal[0]**2+goal[1]**2)

            ####    Weighting Matrices Q R
            if bot_no == 0:
                Q = np.array([[((10*goal_body[0])**2)/abs((z-0.43582)+0.0001)+50/abs(abs(goal_body[0]+0.00001)-1)+100/abs(goal_body[0]+0.00001), 0, 0, 0, 0, 0, 0]
                    ,[0, abs(20*(0.5+abs(goal_body[2]))*(vel_rover[0])/(0.001+0.01*abs(goal_body[0]+0.00001))), 0, 0, 0, 0, 0]
                    ,[0, 0, ((10*goal_body[1])**2)/abs((z-0.43582)+0.0001)+50/abs(abs(goal_body[1]+0.00001)-1)+100/abs(goal_body[1]+0.00001), 0, 0, 0, 0]
                    ,[0, 0, 0, abs(20*(0.5+abs(goal_body[2]))*(vel_rover[1])/(0.001+0.01*abs(goal_body[1]+0.00001))), 0, 0, 0]
                    ,[0, 0, 0, 0, 1+(30*goal_body[2]/sqrt(0.01+0.01*(goal_body[0]**2)+0.01*(goal_body[1]**2)))**2, 0, 0]   #normal
                    ,[0, 0, 0, 0, 0, 1/abs(goal_body[2]+0.001), 0]   #normal
                    ,[0, 0, 0, 0, 0, 0, 10/abs(goal_body[2]+0.001)]])

                R = np.array([[800, 0, 0, 0]    #z - accn
                            ,[0, 75000, 0, 0]   #Pitch
                            ,[0, 0, 75000, 0]   #Roll
                            ,[0, 0, 0, 2000]])

                ## X for vision model as data from camera gives relative states
                if change_landing!=100:
                    X = np.array([[goal_body[0]],[vel_rover[0]],[goal_body[1]],[vel_rover[1]],[goal_body[2]],[vel_rover[2]],[yaw2-yaw]])
                else:
                    X = np.array([[goal_body[0]],[vel_rover[0]-v_x],[goal_body[1]],[vel_rover[1]-v_y],[goal_body[2]],[vel_rover[2]-v_z],[yaw2-yaw]])
            else:
                Q = np.array([[((goal_body[0])**2)/abs(0.5*goal_body[2]**2+0.0001)+10, 0, 0, 0, 0, 0, 0]
                    ,[0, abs(150*(vel_rover[0]-v_x)/(0.001+0.01*abs(goal_body[0])+0.05*abs(goal_body[2]))), 0, 0, 0, 0, 0]
                    ,[0, 0, ((goal_body[1])**2)/abs(0.5*goal_body[2]**2+0.0001)+10, 0, 0, 0, 0]
                    ,[0, 0, 0, abs(150*(vel_rover[1]-v_y)/(0.001+0.01*abs(goal_body[1])+0.05*abs(goal_body[2]))), 0, 0, 0]
                    ,[0, 0, 0, 0, 1+((30*goal_body[2])/sqrt(0.01+0.1*(goal_body[0]**2)+0.1*(goal_body[1]**2)))**2, 0, 0]   #normal
                    # ,[0, 0, 0, 0, 1+((10*goal_body[2]+10*(land))/sqrt(0.01+0.01*(goal_body[0]**2)+0.01*(goal_body[1]**2)))**2, 0, 0]   #alt hold
                    ,[0, 0, 0, 0, 0, 1/abs(goal_body[2]+0.001), 0]   #normal
                    # ,[0, 0, 0, 0, 0, (1-land+0.0001)/abs(goal_body[2]+0.001), 0]   #alt hold
                    ,[0, 0, 0, 0, 0, 0, 10/abs(goal_body[2]+0.001)]])

                R = np.array([[100, 0, 0, 0]    #z - accn
                            ,[0, 100, 0, 0]   #Pitch
                            ,[0, 0, 100, 0]   #Roll
                            ,[0, 0, 0, 500]])

                ## X for baseline model as data we recieve is in global frame
                X = np.array([[goal_body[0]],[vel_rover[0]-v_x],[goal_body[1]],[vel_rover[1]-v_y],[goal_body[2]],[vel_rover[2]-v_z],[yaw2-yaw]])

            B = np.array([[0, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
            P = la.solve_continuous_are(A, B, Q, R)

            u = np.dot(-np.linalg.inv(R),B.transpose())
            u = np.dot(u,P)
            u = np.dot(u,X)

            u0 = float(u[0])
            u1 = float(u[1])
            u2 = float(u[2])
            u3 = float(u[3])

            ####    Normalizing the received thrust
            u0 = ((acc_rover[2]-u0)*1.5 + 14.7)/29.4
            u1 = (acc_rover[0]-u1)/9.8
            u2 = (u2-acc_rover[1])/9.8
            u3 = v3-u3

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

            if abs(yaw2-yaw+2*np.pi)<abs(yaw2-yaw):
                yaw2+=2*np.pi
            if abs(yaw2-yaw-2*np.pi)<abs(yaw2-yaw):
                yaw2-=2*np.pi

            if abs(yaw2-(yaw+u3*0.01))<(abs(yaw2-yaw)):
                pass
            else:
                u3 = -u3

            ####    Start descending for small errors
            if sqrt(goal_body[0]**2+goal_body[1]**2)<0.8 and abs(goal_body[2])<1 and change_landing!=100 and land_flag[bot_no]==1:
                rospy.loginfo("LAND")
                u0 = 0.0
                u1 = 0.0
                u2 = 0.0
            if sqrt(goal_body[0]**2+goal_body[1]**2)<0.8 and abs(goal_body[2])<0.3 and z<1:
                change_landing=100
            if change_landing==100 and z<1:
                armService = rospy.ServiceProxy('drone'+str(bot_no)+'/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
                armService(True)
                takeoffService = rospy.ServiceProxy('drone'+str(bot_no)+'/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
                takeoffService(altitude = 1)


            ####    Convert to quaternions and publish
            quater = tf.transformations.quaternion_from_euler(u2,u1,yaw+np.pi/2) #0
            msg.header = Header()
            msg.type_mask = 0
            msg.orientation.x = quater[0]
            msg.orientation.y = quater[1]
            msg.orientation.z = quater[2]
            msg.orientation.w = quater[3]
            msg.body_rate.x = 0.0
            msg.body_rate.y = 0.0
            msg.body_rate.z = u3
            msg.thrust = u0

            pub.publish(msg)
        rate = rospy.Rate(50)
        rate.sleep

def callback3(info):
    global v_x, v_y, v_z
    v_x = info.twist.twist.linear.x
    v_y = info.twist.twist.linear.y
    v_z = info.twist.twist.linear.z

def callback(info):
    global goal, vel_rover, yaw2, now_p, v1_p, v2_p, acc_rover, v3, x, y, z, roll, pitch, yaw, Rot_body_to_inertial, Rot_inertial_to_body, bot_no, pos_rover

    ## drone_i
    x = info.pose[bot_no+1].position.x
    y = info.pose[bot_no+1].position.y
    z = info.pose[bot_no+1].position.z

    ###     Orientations in order of rotation
    a1 = info.pose[bot_no+1].orientation.x
    b1 = info.pose[bot_no+1].orientation.y
    c1 = info.pose[bot_no+1].orientation.z
    d1 = info.pose[bot_no+1].orientation.w

    roll, pitch, yaw = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])

    Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                                    ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                                    ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
    Rot_inertial_to_body = Rot_body_to_inertial.transpose()

    pos_rover[0] = info.pose[3].position.x
    pos_rover[1] = info.pose[3].position.y

    ## Rover information of yaw and ang, velocity about Z as these cannot be predicted currently even with vision
    a1 = info.pose[3].orientation.x
    b1 = info.pose[3].orientation.y
    c1 = info.pose[3].orientation.z
    d1 = info.pose[3].orientation.w

    r, p, yaw2 = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])
    v3 = info.twist[3].angular.z

    if change_landing==100:
        goal[0] = 20
        goal[1] = 20
        goal[2] = 10*(bot_no+1)
        vel_rover[0] = 0.0
        vel_rover[1] = 0.0
        vel_rover[2] = 0.0
        acc_rover[0] = 0.0
        acc_rover[1] = 0.0
        acc_rover[2] = 0.0
    elif bot_no!=0:
        now = rospy.get_time()
        if now-now_p == 0:
            pass
        else:
            ##      Receive position info
            goal[0] = info.pose[3].position.x
            goal[1] = info.pose[3].position.y
            goal[2] = 0.435

            ##      Receive vel info and convert to body fixed frame
            v1 = info.twist[3].linear.x
            v2 = info.twist[3].linear.y
            v3 = info.twist[3].angular.z

            a1 = (v1-v1_p)/(now-now_p)
            a2 = (v2-v2_p)/(now-now_p)
            now_p = timer()

            v = np.array([[v1]
                        ,[v2]
                        ,[0.0]])

            a = np.array([[a1]
                        ,[a2]
                        ,[0.0]])

            v = np.dot(Rot_inertial_to_body, v)
            a = np.dot(Rot_inertial_to_body, a)

            vel_rover[0] = float(v[0])
            vel_rover[1] = float(v[1])
            vel_rover[2] = float(v[2])
            acc_rover[0] = float(a[0])
            acc_rover[1] = float(a[1])
            acc_rover[2] = float(a[2])
            v1_p = v1
            v2_p = v2

def ReceiveTar(info):
    global goal, vel_rover, Rot_inertial_to_body, now, now_p2, detect

    ##      Receive position info
    goal[0] = info.goal.x
    goal[1] = info.goal.y
    goal[2] = 0.435-z
    detect = info.detected

    ##      Receive vel info and convert to body fixed frame
    v1 = info.vel.x
    v2 = info.vel.y
    v = np.array([[v1]
                ,[v2]
                ,[0.0-v_z]])
    v = np.dot(Rot_inertial_to_body, v)
    vel_rover[0] = float(v[0])
    vel_rover[1] = float(v[1])
    vel_rover[2] = float(v[2])

def callback2(info):
    global land_flag
    land_flag = info.land

def listener():
    global bot_no
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    rospy.Subscriber("/landing_info", land, callback2)
    rospy.Subscriber("/drone1/mavros/local_position/odom", Odometry, callback3)
    if bot_no == 0:
        rospy.Subscriber('/kalman_filter', kalman, ReceiveTar)
    sdre()
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        # posn.close()
        pass
