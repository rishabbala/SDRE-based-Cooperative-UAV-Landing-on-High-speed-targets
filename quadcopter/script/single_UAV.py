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

# #########################################################################################################################################################
#
# This is the code for landing a single UAV on a moving target with/without coordination
# It is inbuilt with code for rover trajectory and flag changes
#
# #########################################################################################################################################################


## Initialize node and publisher to UAV attitude and rover velocity topics
rospy.init_node('sdre', anonymous=True)
pub = rospy.Publisher("/drone0/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)
pub2 = rospy.Publisher("/rover/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)

## Getting plots
posn = open('tight_square.csv', 'w')

## Drone orientation
roll = 0.0
pitch = 0.0
yaw = 0.0
## Rover orientations
yaw2 = 0.0
detect = 0
now = rospy.get_time()
now_p = rospy.get_time()

msg = AttitudeTarget()
msg2 = Twist()

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

pos_rover = [0,0,0]
acc_rover = [0,0,0]
vel_rover = [0,0,0]

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

## Trajectory for rover
# rover_goal = [[3,0]
#              ,[3,3]
#              ,[6,3]
#              ,[6,-3]
#              ,[9,-3]
#              ,[9,3]
#              ,[12,3]
#              ,[12,-3]
#              ,[15,-3]
#              ,[15,3]
#              ,[18,3]
#              ,[18,-3]
#              ,[21,-3]
#              ,[21,3]
#              ,[24,3]
#              ,[24,-3]
#              ,[27,-3]
#              ,[27,3]
#              ,[30,3]
#              ,[30,-3]
#              ,[33,-3]
#              ,[33,3]
#              ,[36,3]
#              ,[36,-3]
#              ,[39,-3]
#              ,[39,3]
#              ,[42,3]
#              ,[42,-3]
#              ,[45,-3]
#              ,[48,3]
#              ,[51,3]
#              ,[51,-3]]

rover_goal = [[8,0]
             ,[8,4]
             ,[-8,4]
             ,[-8,-0]
             ,[8,0]
             ,[8,4]
             ,[-8,4]
             ,[-8,-0]
             ,[8,0]
             ,[8,4]
             ,[-8,4]
             ,[-8,-0]
             ,[8,0]
             ,[8,4]
             ,[-8,4]
             ,[-8,-0]
             ,[8,0]
             ,[8,4]
             ,[-8,4]
             ,[-8,-0]
             ,[8,0]
             ,[8,4]
             ,[-8,4]
             ,[-8,-0]
             ,[8,0]
             ,[8,4]
             ,[-8,4]
             ,[-8,-0]
             ,[8,0]
             ,[8,4]
             ,[-8,4]
             ,[-8,-0]]

flag2=0
land = 0
zhold = 10 # initial height, can also be taken as input
def sdre():
    while not rospy.is_shutdown():
        global flag2, x, y, z, roll, pitch, yaw, vel_rover, goal, goal_body, v_x, v_y, v_z, Rot_body_to_inertial, Rot_inertial_to_body, yaw2, acc_rover, v3, zhold, rover_goal, pos_rover, land

        ## For plots
        posn.write('%f;' % float(x))
        posn.write('%f;' % float(y))
        posn.write('%f;' % float(z))
        posn.write('%f;' % float(v_x))
        posn.write('%f;' % float(v_y))
        posn.write('%f;' % float(v_z))
        posn.write('%f;' % float(pos_rover[0]))
        posn.write('%f;' % float(pos_rover[1]))
        posn.write('%f;' % float(pos_rover[2]))
        posn.write('%f;' % float(vel_rover[0]))
        posn.write('%f;' % float(vel_rover[1]))
        posn.write('%f;' % float(vel_rover[2]))
        posn.write('\n')

        ## If height > 2, follow form behind the rover, dont go too close
        if z>4:
            goal[0] = goal[0] - 2*cos(yaw2)*(1/(1+exp(-(z-6))))
            goal[1] = goal[1] - 2*sin(yaw2)*(1/(1+exp(-(z-6))))

        ## Hold position if not landing scenario
        if land==1:
           goal[2] = zhold

        print(goal, land)

        ## Goals are the errors here
        goal_body[0] = goal[0] - x
        goal_body[1] = goal[1] - y
        goal_body[2] = goal[2] - z

        ## The errors in body frame
        goal_body = np.dot(Rot_inertial_to_body,goal_body.transpose())

        #  !!!! TEST
        # If mislanded, especially vision based, start regaining altitude
        if (sqrt(goal_body[0]**2 + goal_body[1]**2)>1 and land==0) or (detect==0):
            goal[2] = sqrt(goal[0]**2+goal[1]**2)

        ####    Weighting Matrices Q R

################################################################################

########        USE THESE MATRICES FOR BASELINE CONTROL      ########
        Q = np.array([[((goal_body[0])**2)/abs(0.5*goal_body[2]**2+0.0001)+10, 0, 0, 0, 0, 0, 0]
            ,[0, abs(150*(vel_rover[0]-v_x)/(0.001+0.01*abs(goal_body[0])+0.05*abs(goal_body[2]))), 0, 0, 0, 0, 0]
            ,[0, 0, ((goal_body[1])**2)/abs(0.5*goal_body[2]**2+0.0001)+10, 0, 0, 0, 0]
            ,[0, 0, 0, abs(150*(vel_rover[1]-v_x)/(0.001+0.01*abs(goal_body[1])+0.05*abs(goal_body[2]))), 0, 0, 0]
            ,[0, 0, 0, 0, 1+((30*goal_body[2])/sqrt(0.01+0.1*(goal_body[0]**2)+0.1*(goal_body[1]**2)))**2, 0, 0]   #normal
            # ,[0, 0, 0, 0, 1+((10*goal_body[2]+10*(land))/sqrt(0.01+0.01*(goal_body[0]**2)+0.01*(goal_body[1]**2)))**2, 0, 0]   #alt hold
            ,[0, 0, 0, 0, 0, 1/abs(goal_body[2]+0.001), 0]   #normal
            # ,[0, 0, 0, 0, 0, (1-land+0.0001)/abs(goal_body[2]+0.001), 0]   #alt hold
            ,[0, 0, 0, 0, 0, 0, 10/abs(goal_body[2]+0.001)]])

        R = np.array([[100, 0, 0, 0]    #z - accn
                    ,[0, 100, 0, 0]   #Pitch
                    ,[0, 0, 100, 0]   #Roll
                    ,[0, 0, 0, 300]])

        print(Q)
################################################################################


################################################################################

##########          USE THIS FOR VISION BASED CONTROL   ########################
        # Q = np.array([[((10*goal_body[0])**2)/abs(goal_body[2]+0.0001)+50/abs(abs(goal_body[0]+0.00001)-1)+100/abs(goal_body[0]+0.00001), 0, 0, 0, 0, 0, 0]
        #     ,[0, abs(20*(0.5+abs(goal_body[2]))*(vel_rover[0])/(0.001+0.01*abs(goal_body[0]+0.00001))), 0, 0, 0, 0, 0]
        #     ,[0, 0, ((10*goal_body[1])**2)/abs(goal_body[2]+0.0001)+50/abs(abs(goal_body[1]+0.00001)-1)+100/abs(goal_body[1]+0.00001), 0, 0, 0, 0]
        #     ,[0, 0, 0, abs(20*(0.5+abs(goal_body[2]))*(vel_rover[1])/(0.001+0.01*abs(goal_body[1]))), 0, 0, 0]
        #     ,[0, 0, 0, 0, 1+(30*goal_body[2]/sqrt(0.01+0.01*(goal_body[0]**2)+0.01*(goal_body[1]**2)))**2, 0, 0]   #normal
        #     ,[0, 0, 0, 0, 0, 1/abs(goal_body[2]+0.001), 0]   #normal
        #     ,[0, 0, 0, 0, 0, 0, 10/abs(goal_body[2]+0.001)]])
        #
        # R = np.array([[800, 0, 0, 0]    #z - accn
        #             ,[0, 75000, 0, 0]   #Pitch
        #             ,[0, 0, 75000, 0]   #Roll
        #             ,[0, 0, 0, 2000]])


################################################################################

        ## Calculation for control done in body fixed frame
        ## d2(e_x)/dt2 = 0-d2(x)/dt2 so all signs inverted

        ## X for baseline model as data we recieve is in global frame
        X = np.array([[goal_body[0]],[vel_rover[0]-v_x],[goal_body[1]],[vel_rover[1]-v_y],[goal_body[2]],[vel_rover[2]-v_z],[yaw2-yaw]])
        ## X for vision model as data from camera gives relative states
        # X = np.array([[goal_body[0]],[vel_rover[0]],[goal_body[1]],[vel_rover[1]],[goal_body[2]],[vel_rover[2]],[yaw2-yaw]])
        B = np.array([[0, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
        P = la.solve_continuous_are(A, B, Q, R)

        ## Riccati Equation solving
        u = np.dot(-np.linalg.inv(R),B.transpose())
        u = np.dot(u,P)
        u = np.dot(u,X)

        ## Inputs thrust, roll, pitch, and yaw angular velocity
        u0 = float(u[0])
        u1 = float(u[1])
        u2 = float(u[2])
        u3 = float(u[3])

        ## Normalizing the received thrust
        u0 = ((acc_rover[2]-u0)*1.5 + 14.7)/29.4
        u1 = (acc_rover[0]-u1)/9.8
        u2 = (u2-acc_rover[1])/9.8
        u3 = v3-u3

        ## Making thrust non-negative
        if u0>1:
            u0 = 1
        if u0<0:
            u0 = 0

        ## Restrict rotation angles to 10 deg
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

        # ## Restricting rotation Rate
        # if u3>0.8:
        #     u3 = 0.8
        # if u3<-0.8:
        #     u3 = -0.8

        ## Start descending for small errors
        if sqrt(goal_body[0]**2+goal_body[1]**2)<0.8 and abs(goal_body[2])<1 and land==0:
            rospy.loginfo("LAND")
            u0 = 0.0
            u1 = 0.0
            u2 = 0.0


        ## Convert to quaternions and publish
        quater = tf.transformations.quaternion_from_euler(u2,u1,yaw+np.pi/2) ## yaw+pi/2 only if mavros is used !!!!
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

        ## Rover trajectory
        ## Change goal of rover to next waypoint
        if sqrt((pos_rover[0]-rover_goal[flag2][0])**2 + (pos_rover[1]-rover_goal[flag2][1])**2) <= 2 and flag2!=len(rover_goal)-1:
            flag2+=1
            ## Based on flag2 decide to land or track
            if flag2%2 == 0:
                land = 1
                zhold = z
            else:
                land = 0

        ## Desired Heading and calculation of error in heading
        ang = atan2(rover_goal[flag2][1]-pos_rover[1],rover_goal[flag2][0]-pos_rover[0])
        if abs(ang-yaw2+2*np.pi)<abs(ang-yaw2):
            ang+=2*np.pi
        if abs(ang-yaw2-2*np.pi)<abs(ang-yaw2):
            ang-=2*np.pi

        ## If desired heaing is too far off from current heading, dont land as rover is going to rotate
        if abs(ang-yaw2)>=0.78539:
            land = 1
            zhold = z
        elif flag2%2 != 0:
            land = 0

        ## Turning if heading error > 0.3rad
        if abs(ang-yaw2)>=0.3:
            ## Choose direction of rotation that reduces error
            if abs(ang-(yaw2+(ang-yaw2)*0.01))<abs(ang-yaw2):
                msg2.angular.z = 0.5*(ang-yaw2)
            else:
                msg2.angular.z = -0.5*(ang-yaw2)
            msg2.linear.x = 1
        else:
            ## Dont land constraints. This constraint is set by us to check effectiveness
            if flag2%2==0:
                land=1
            else:
                land=0
            msg2.linear.x = 1.5*(sqrt((pos_rover[0]-rover_goal[flag2][0])**2 + (pos_rover[1]-rover_goal[flag2][1])**2))
            if abs(ang - (yaw2+(ang-yaw2)*0.001))<abs(ang-yaw2):
                msg2.angular.z = 0.8*(ang - yaw2)
            else:
                msg2.angular.z = -0.8*(ang - yaw2)

        ## Bounding the velocities
        if msg2.angular.z>0.8:
            msg2.angular.z = 0.8
        if msg2.angular.z<-0.8:
            msg2.angular.z = -0.8
        if msg2.linear.x>1.5:
            msg2.linear.x = 1.5

        pub2.publish(msg2)
        rate = rospy.Rate(50)
        rate.sleep

################################################################################

# USE THIS ONLY IF YOU WANT DATA FROM MAVROS AND NOT GAZEBO

def callback(info):
    ##MUST GET HEADING
    global x, y, z, roll, pitch, yaw, vel_rover, vel_drone_rot, vel_drone_trans, head, error_head_prev, goal, goal_body, v_x, v_y, v_z, Rot_body_to_inertial, Rot_inertial_to_body

    ## Positions in global gazebo frame
    x = info.pose.pose.position.y
    y = -info.pose.pose.position.x
    z = info.pose.pose.position.z

    ## All linear velocities are local
    v_x = info.twist.twist.linear.x
    v_y = info.twist.twist.linear.y
    v_z = info.twist.twist.linear.z

    ## Orientations in order of rotation
    a1 = info.pose.pose.orientation.x
    b1 = info.pose.pose.orientation.y
    c1 = info.pose.pose.orientation.z
    d1 = info.pose.pose.orientation.w

    # roll, pitch, yaw = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])

    ###     Yaw in gazebo frame
    # yaw = yaw-np.pi/2

    Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                                    ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                                    ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
    Rot_inertial_to_body = Rot_body_to_inertial.transpose()
################################################################################


################################################################################

# ONLY FOR THE BASELINE MODEL

def callback2(info):
    global x, y, z, v_x, v_y, v_z, roll, pitch, yaw, goal, vel_rover, acc_rover, Rot_body_to_inertial, Rot_inertial_to_body, yaw2, now_p, v1_p, v2_p, v3, pos_rover
    now = rospy.get_time()

    if now-now_p == 0:
        pass
    else:
    ############################################################################
        # USE THIS ONLY IF YOU WANT TO GET DATA FROM GAZEBO, VELOCITY MUST BE CHECKED IF IN BODY FRAME

        # ## Drone information
        # x = info.pose[1].position.x
        # y = info.pose[1].position.y
        # z = info.pose[1].position.z
        #
        # v_x = info.twist[1].linear.x
        # v_y = info.twist[1].linear.y
        # v_z = info.twist[1].linear.z
        #
        # a1 = info.pose[1].orientation.x
        # b1 = info.pose[1].orientation.y
        # c1 = info.pose[1].orientation.z
        # d1 = info.pose[1].orientation.w
        # roll, pitch, yaw = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])
        #
        # Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
        #                                 ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
        #                                 ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
        # Rot_inertial_to_body = Rot_body_to_inertial.transpose()

    ############################################################################

        ## Drone orientations alone
        a1 = info.pose[1].orientation.x
        b1 = info.pose[1].orientation.y
        c1 = info.pose[1].orientation.z
        d1 = info.pose[1].orientation.w
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])

        ## Rover information
        pos_rover[0] = info.pose[2].position.x
        pos_rover[1] = info.pose[2].position.y
        goal[0] = info.pose[2].position.x+8 ## adding initial UAV position to get in mavros frame
        goal[1] = info.pose[2].position.y+0 ## adding initial UAV position to get in mavros frame
        goal[2] = 0.435

        a1 = info.pose[2].orientation.x
        b1 = info.pose[2].orientation.y
        c1 = info.pose[2].orientation.z
        d1 = info.pose[2].orientation.w
        roll2, pitch2, yaw2 = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])

        ## Receive vel info and convert to body fixed frame
        v1 = info.twist[2].linear.x
        v2 = info.twist[2].linear.y
        v3 = info.twist[2].angular.z

        ## Calculate acceleration
        a1 = (v1-v1_p)/(now-now_p)
        a2 = (v2-v2_p)/(now-now_p)
        now_p = rospy.get_time()

        v = np.array([[v1]
                    ,[v2]
                    ,[0.0]])

        a = np.array([[a1]
                    ,[a2]
                    ,[0.0]])

        ## Converting from W to B
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
################################################################################

################################################################################

# ONLY FOR VISION MODEL

# def ReceiveTar(info):
#     global goal, vel_rover, Rot_inertial_to_body, now, now_p2, detect
#
#     ##      Receive position info
#     goal[0] = info.goal.x
#     goal[1] = info.goal.y
#     goal[2] = 0.435-z
#     detect = info.detected
#
#     ##      Receive vel info and convert to body fixed frame
#     v1 = info.vel.x
#     v2 = info.vel.y
#     v = np.array([[v1]
#                 ,[v2]
#                 ,[0.0-v_z]])
#     v = np.dot(Rot_inertial_to_body, v)
#     vel_rover[0] = float(v[0])
#     vel_rover[1] = float(v[1])
#     vel_rover[2] = float(v[2])

################################################################################

def listener():
    # rospy.Subscriber('/kalman_filter', kalman, ReceiveTar)         ## If using vision
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback2) ## If baseline model
    rospy.Subscriber("/drone0/mavros/local_position/odom", Odometry, callback)
    sdre()
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        posn.close()
        pass
