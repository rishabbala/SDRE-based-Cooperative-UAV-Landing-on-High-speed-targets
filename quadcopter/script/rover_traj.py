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

rospy.init_node('rover_traj', anonymous=True)
pub2 = rospy.Publisher("/rover/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
pub = rospy.Publisher("/landing_info", land, queue_size=1)

posn = open('oscillate_multi_coord.csv', 'w')

msg2 = Twist()
msg = land()
pos_rover = [0,0,0]
pos_drone = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
yaw2 = 0
flag2 = 0

land_flag = [1,0]
pos = 0

# rover_goal = [[8,0]
#              ,[8,4]
#              ,[-8,4]
#              ,[-8,-0]
#              ,[8,0]
#              ,[8,4]
#              ,[-8,4]
#              ,[-8,-0]
#              ,[8,0]
#              ,[8,4]
#              ,[-8,4]
#              ,[-8,-0]
#              ,[8,0]
#              ,[8,4]
#              ,[-8,4]
#              ,[-8,-0]
#              ,[8,0]
#              ,[8,4]
#              ,[-8,4]
#              ,[-8,-0]
#              ,[8,0]
#              ,[8,4]
#              ,[-8,4]
#              ,[-8,-0]
#              ,[8,0]
#              ,[8,4]
#              ,[-8,4]
#              ,[-8,-0]
#              ,[8,0]
#              ,[8,4]
#              ,[-8,4]
#              ,[-8,-0]]

rover_goal = [[3,0]
             ,[3,3]
             ,[6,3]
             ,[6,-3]
             ,[9,-3]
             ,[9,3]
             ,[12,3]
             ,[12,-3]
             ,[15,-3]
             ,[15,3]
             ,[18,3]
             ,[18,-3]
             ,[21,-3]
             ,[21,3]
             ,[24,3]
             ,[24,-3]
             ,[27,-3]
             ,[27,3]
             ,[30,3]
             ,[30,-3]
             ,[33,-3]
             ,[33,3]
             ,[36,3]
             ,[36,-3]
             ,[39,-3]
             ,[39,3]
             ,[42,3]
             ,[42,-3]
             ,[45,-3]
             ,[48,3]
             ,[51,3]
             ,[51,-3]]

zhold = 10
temp = []

def trajectory_gen():
    while not rospy.is_shutdown():
        global pos_rover, yaw2, flag2, land_flag, pos_drone, land_flag, pos, zhold
        if len(pos_drone)==2 and pos_rover!=[]:
            if pos+1<len(land_flag):
                if sqrt((pos_drone[pos][0]-pos_rover[0])**2 + (pos_drone[pos][1]-pos_rover[1])**2)<0.8 and abs(pos_drone[pos][2]-0.435)<0.25:
                    land_flag[pos] = 0
                    pos+=1
                    land_flag[pos] = 1

            ang = atan2(rover_goal[flag2][1]-pos_rover[1],rover_goal[flag2][0]-pos_rover[0])
            if abs(ang-yaw2+2*np.pi)<abs(ang-yaw2):
                ang+=2*np.pi
            if abs(ang-yaw2-2*np.pi)<abs(ang-yaw2):
                ang-=2*np.pi

            if sqrt((pos_rover[0]-rover_goal[flag2][0])**2 + (pos_rover[1]-rover_goal[flag2][1])**2) <= 2:
                if flag2!=len(rover_goal)-1:
                    flag2+=1
                else:
                    flag2 = 0
                if flag2%2==0 or abs(ang-yaw2)>=0.4 or flag2 == 0 or flag2 == 1:
                    zhold = pos_drone[pos][2]
                    land_flag[pos]=10
                else:
                    land_flag[pos] = 1

            if abs(ang-yaw2+2*np.pi)<abs(ang-yaw2):
                ang+=2*np.pi
            if abs(ang-yaw2-2*np.pi)<abs(ang-yaw2):
                ang-=2*np.pi
            if abs(ang-yaw2)>=0.3:
                if abs(ang-(yaw2+(ang-yaw2)*0.001))<abs(ang-yaw2):
                    msg2.angular.z = 0.5*(ang-yaw2)
                else:
                    msg2.angular.z = -0.5*(ang-yaw2)
                msg2.linear.x = 0.5
            else:
                msg2.linear.x = 1.5*(sqrt((pos_rover[0]-rover_goal[flag2][0])**2 + (pos_rover[1]-rover_goal[flag2][1])**2))
                if abs(ang - (yaw2+(ang-yaw2)*0.001))<abs(ang-yaw2):
                    msg2.angular.z = 0.8*(ang - yaw2)
                else:
                    msg2.angular.z = -0.8*(ang - yaw2)

            if msg2.angular.z>0.5:
                msg2.angular.z = 0.5
            if msg2.angular.z<-0.5:
                msg2.angular.z = -0.5
            if msg2.linear.x>1.0:
                msg2.linear.x = 1.0

            temp = [land_flag[0], land_flag[1], zhold]
            msg.land = temp

            print(flag2)
            pub.publish(msg)
            pub2.publish(msg2)
        rate = rospy.Rate(50)
        rate.sleep

def callback2(info):
    global yaw2, pos_rover, pos_drone
    pos_drone[0][0] = info.pose[1].position.x
    pos_drone[0][1] = info.pose[1].position.y
    pos_drone[0][2] = info.pose[1].position.z
    pos_drone[0][3] = info.twist[1].linear.x
    pos_drone[0][4] = info.twist[1].linear.y
    pos_drone[0][5] = info.twist[1].linear.z

    pos_drone[1][0] = info.pose[2].position.x
    pos_drone[1][1] = info.pose[2].position.y
    pos_drone[1][2] = info.pose[2].position.z
    pos_drone[1][3] = info.twist[2].linear.x
    pos_drone[1][4] = info.twist[2].linear.y
    pos_drone[1][5] = info.twist[2].linear.z
    for i in range(2):
    #     pos_drone.append([info.pose[i+1].position.x, info.pose[i+1].position.y, info.pose[i+1].position.z, info.twist[i+1].linear.x, info.twist[i+1].linear.y, info.twist[i+1].linear.z])
        posn.write('%f;' % float(pos_drone[i][0]))
        posn.write('%f;' % float(pos_drone[i][1]))
        posn.write('%f;' % float(pos_drone[i][2]))
        posn.write('%f;' % float(pos_drone[i][3]))
        posn.write('%f;' % float(pos_drone[i][4]))
        posn.write('%f;' % float(pos_drone[i][5]))
    pos_rover[0] = info.pose[3].position.x
    pos_rover[1] = info.pose[3].position.y
    pos_rover[2] = 0.4325

    posn.write('%f;' % float(pos_rover[0]))
    posn.write('%f;' % float(pos_rover[1]))
    posn.write('%f;' % float(pos_rover[2]))

    # vel_rover[0] = info.twist[3].linear.x
    # vel_rover[1] = info.twist[3].linear.y
    # vel_rover[2] = 0.0

    posn.write('%f;' % float(info.twist[3].linear.x))
    posn.write('%f;' % float(info.twist[3].linear.y))
    posn.write('%f;' % float(info.twist[3].linear.z))
    posn.write('\n')

    a1 = info.pose[3].orientation.x
    b1 = info.pose[3].orientation.y
    c1 = info.pose[3].orientation.z
    d1 = info.pose[3].orientation.w
    r, p, yaw2 = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])

def listener():
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback2)
    trajectory_gen()
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
