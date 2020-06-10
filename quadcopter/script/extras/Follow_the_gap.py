#!/usr/bin/env python
import rospy
import tf
import numpy as np
from math import *
from sensor_msgs.msg import LaserScan
from numpy import inf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Twist, PoseStamped

rospy.init_node('obstacle_avoidance', anonymous=True)
pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)

roll = 0.0
pitch = 0.0
yaw = 0.0
x = 0.0
y = 0.0
z = 0.0
error_ang_sum = 0.0
error_ang_prev = 0.0
error_dist_prev = 0.0
error_dist_sum = 0.0
twist_flag = 0
corner = []
l = []
direction = []
virt_goal = []
goal2 = []
mode = []
flag = 0
mean = np.array([[0.0 for j in range(720)]for k in range (3)])
d_flag = 0
obst_flag = 0
d_min = 2
#Gazebo_x = Ardu_y      Gazebo_y = -Ardu_x
# Go to 10,-10 in mavros ==> 10,10 in Gazebo
goal = [10, 10]
f = 0
pos = 0

def get_obst(range_data):
    global yaw, obst_flag, l, d_min, pos
    pos = 0
    yaw2 = yaw
    d_min = 2
    g = atan2((goal[1]-y),(goal[0]-x))
    if g<-np.pi:
        g+=2*np.pi
    if g>np.pi:
        g-=2*np.pi

    if yaw2<-np.pi:
        yaw2+=2*np.pi
    if yaw2>np.pi:
        yaw2-=2*np.pi
    m1 = yaw2-g

    if m1<-np.pi:
        m1+=2*np.pi
    if m1>np.pi:
        m1-=2*np.pi

    x1 = x+0.5*cos(yaw+0.51557906)
    y1 = y+0.5*sin(yaw+0.51557906)
    x2 = x+0.5*cos(yaw-0.51557906)
    y2 = y+0.5*sin(yaw-0.51557906)
    x3 = x+0.5*cos(yaw)
    y3 = y+0.5*sin(yaw)
    
    #rospy.loginfo("ANGLE %s %s %s", abs(m1), yaw, g)
    for i in range(240,480): 
        if abs(range_data[i]*cos(-1.57079994678 + 0.00436940183863*i))<2 and abs(range_data[i]*sin(-1.57079994678 + 0.00436940183863*i))<1 and abs(m1)<np.pi/2:
            if dist(get_coords([range_data[i], -1.57079994678 + 0.00436940183863*i]), [x1,y1])<d_min or dist(get_coords([range_data[i], -1.57079994678 + 0.00436940183863*i]), [x2,y2])<d_min or dist(get_coords([range_data[i], -1.57079994678 + 0.00436940183863*i]), [x3,y3])<d_min:
                d_min = min([dist(get_coords([range_data[i], -1.57079994678 + 0.00436940183863*i]), [x1,y1]), dist(get_coords([range_data[i], -1.57079994678 + 0.00436940183863*i]), [x2,y2]), dist(get_coords([range_data[i], -1.57079994678 + 0.00436940183863*i]), [x3,y3])])
                pos = i
            obst_flag = 1
            #rospy.loginfo("OBSTACLE")
            l.append(i)


def get_coords(m1):
    global x, y, yaw
    x1 = m1[0]*cos(m1[1])
    y1 = m1[0]*sin(m1[1])
    x_closest = -y1*sin(yaw) + x1*cos(yaw) + x
    y_closest = y1*cos(yaw) + x1*sin(yaw) + y
    return [x_closest, y_closest]

def transform(info):
    global roll, pitch, yaw
    global x, y, z
    x = info.pose.position.y
    y = -info.pose.position.x
    z = info.pose.position.z
    a = info.pose.orientation.x
    b = info.pose.orientation.y
    c = info.pose.orientation.z
    d = info.pose.orientation.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([a,b,c,d])
    yaw = -(-yaw+1.5707999)   #####    NED --->  ENU Check quaternions

def dist(a,b):
    distance = sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    return distance



def scan(range_data):
    range_data = list(range_data)
    co1 = 0
    co2 = 0
    global corner, direction, virt_goal
    corner = []
    direction = []
    for i in range(1,len(range_data)-1):  ##       2m in from 0.25m to side
        if i == 1 and range_data[i]<30:
            corner.append([range_data[i], -1.57079994678 + 0.00436940183863*i, i])
            direction.append(0)
        elif i == len(range_data)-1 and range_data[i]<30:
            corner.append([range_data[i], -1.57079994678 + 0.00436940183863*i, i])
            direction.append(1)

        if abs(range_data[i]-range_data[i+1])>3:
            if range_data[i+1]>range_data[i]: 
                direction.append(1)  #### 1 = RIGHT     0 = LEFT
                corner.append([range_data[i], -1.57079994678 + 0.00436940183863*i, i])
            else:
                direction.append(0)
                corner.append([range_data[i+1], -1.57079994678 + 0.00436940183863*i, i])
                

    #rospy.loginfo("INSIDE %s, %s, %s", corner, direction, d_flag) 
    rospy.loginfo("SCANNED %s %s",len(corner),corner)
    

def cluster(range_data):
    global corner, direction, mode
    mode = []
    temp = []

    rospy.loginfo("CORNER %s %s",corner, len(corner))

    for i in range(0,len(direction)/2-1):
        if dist(get_coords([corner[2*i+1][0],corner[2*i+1][1]]), get_coords([corner[2*i+2][0],corner[2*i+2][1]]))<2:
            temp.append(2*i+1)
            temp.append(2*i+2)
    
    for i in range(len(temp)):
        del corner[temp[i]-i]
        del direction[temp[i]-i]



def ret_head():
    global x, y, z, goal2, goal
    if goal2 == []:
        head_reqd = atan2((goal[1]-y),(goal[0]-x))
        if head_reqd>np.pi:
            head_reqd = head_reqd - 2*np.pi
        if head_reqd<-np.pi:
            head_reqd = head_reqd + 2*np.pi
        error_dist = dist(goal,[x,y])
    else:
        head_reqd = atan2((goal2[1]-y),(goal2[0]-x))
        if head_reqd>np.pi:
            head_reqd = head_reqd - 2*np.pi
        if head_reqd<-np.pi:
            head_reqd = head_reqd + 2*np.pi
        error_dist = dist(goal2,[x,y])
    return head_reqd, error_dist


def min_cost(info):
    global x, y, goal
    d_min = dist(goal,info) + dist([x,y],info)
    return d_min


def callback(data):
    r = data.ranges
    r = list(r)
    for i in range(len(r)):
        if r[i]!=inf:
            r[i] = r[i]-(0.2/abs(cos(-1.57079994678 + 0.00436940183863*i)))
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.z = 0.0
    temp = np.array([0.0 for j in range(720)])
    temp = np.reshape(temp, (1,720))
    ####  3 rows 720 coloumns
    global f, d_min, flag, mean, corner, direction, x, y, z, yaw, obst_flag, l, mode, error_ang_prev, error_dist_prev, goal2, error_ang_sum, pos
    g = []
    l = []
    obst_flag = 0
    corner = []
    direction = []

    ################        APPLY MEAN/MEDIAN
    if flag<=2:
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.z = 0.0
        rospy.loginfo("DOING")
        for i in range(len(r)):
            if i!=0 and i!= len(r)-1:
                mean[flag][i] = (r[i-1] + r[i] + r[i+1])/3
            elif i ==0:
                mean[flag][i] = (r[i] + r[i+1])/2
            elif i ==len(r)-1:
                mean[flag][i] = (r[i] + r[i-1])/2

    ##############      APPLIED
    else:
        rospy.loginfo("DONE")
        mean = np.delete(mean, (0), axis=0)
        for i in range(len(r)):
            if i!=0 and i!= len(r)-1:
                temp[0][i] = (r[i-1] + r[i] + r[i+1])/3
            elif i ==0:
                temp[0][i] = (r[i] + r[i+1])/2
            elif i ==len(r)-1:
                temp[0][i] = (r[i] + r[i-1])/2
        mean = np.append(mean, temp, axis = 0)

        #r = list(r)
        r = np.median(mean, axis = 0)
        

        alpha, error_dist = ret_head()
        if goal2!=[]:
            if dist([x,y], goal2) < 0.8:
                goal2 = goal
                rospy.loginfo("CHANGED")
        theta = alpha

        get_obst(r)
        if len(l)>0:
            scan(r)
            cluster(r)
            
            temp = 0
            
            x1 = x+0.5*cos(yaw+0.51557906)
            y1 = y+0.5*sin(yaw+0.51557906)
            x2 = x+0.5*cos(yaw-0.51557906)
            y2 = y+0.5*sin(yaw-0.51557906)
            x3 = x+0.5*cos(yaw)
            y3 = y+0.5*sin(yaw)
            
            
            for j in range(len(corner)/2):
                if corner[j][2]<=pos<=corner[j+1][2] and j not in g:
                    g = [j, j+1]

            rospy.loginfo("G%s", g)
            
            co1 = get_coords([corner[g[0]][0], corner[g[0]][1]])
            co2 = get_coords([corner[g[1]][0], corner[g[1]][1]])

            if dist(co1, [x1,y1]) + dist(co1, [x2,y2]) + dist(co1, [x3,y3])<dist(co2, [x1,y1]) + dist(co2, [x2,y2]) + dist(co2, [x3,y3]):
                if -np.pi/2<yaw<np.pi/2:
                    if direction[g[0]] == 0:
                        d = sqrt(dist([x,y],co1)**2 + 1.5**2)
                        theta = corner[g[0]][1]-abs(atan2(1.5,dist([x,y],co1)))
                        go = get_coords([d,theta])
                    else:
                        d = sqrt(dist([x,y],co1)**2 + 1.5**2)
                        theta = corner[g[0]][1]+abs(atan2(1.5,dist([x,y],co1)))
                        go = get_coords([d,theta])
                else:
                    if direction[g[0]] == 0:
                        d = sqrt(dist([x,y],co1)**2 + 1.5**2)
                        theta = corner[g[0]][1]-abs(atan2(1.5,dist([x,y],co1)))
                        go = get_coords([d,theta])
                    else:
                        d = sqrt(dist([x,y],co1)**2 + 1.5**2)
                        theta = corner[g[0]][1]+abs(atan2(1.5,dist([x,y],co1)))
                        go = get_coords([d,theta])

            else:
                if -np.pi/2<yaw<np.pi/2:
                    if direction[g[1]] == 0:
                        d = sqrt(dist([x,y],co2)**2 + 1.5**2)
                        theta = corner[g[1]][1]-abs(atan2(1.5,dist([x,y],co2)))
                        go = get_coords([d,theta])
                    else:
                        d = sqrt(dist([x,y],co2)**2 + 1.5**2)
                        theta = corner[g[1]][1]+abs(atan2(1.5,dist([x,y],co2)))
                        go = get_coords([d,theta])
                else:
                    if direction[g[1]] == 0:
                        d = sqrt(dist([x,y],co2)**2 + 1.5**2)
                        theta = corner[g[1]][1]-abs(atan2(1.5,dist([x,y],co2)))
                        go = get_coords([d,theta])
                    else:
                        d = sqrt(dist([x,y],co2)**2 + 1.5**2)
                        theta = corner[g[1]][1]+abs(atan2(1.5,dist([x,y],co2)))
                        go = get_coords([d,theta])

            if theta<-np.pi:
                theta+=2*np.pi
            if theta>np.pi:
                theta-=2*np.pi
            goal2 = go
        rospy.loginfo("GOAL2 %s",goal2)
        error_ang = theta-yaw

        if error_ang>np.pi:
            error_ang = error_ang-2*np.pi
        if error_ang<-np.pi:
            error_ang = error_ang+2*np.pi
        
        msg.angular.z = (1*error_ang + 0.1*(error_ang-error_ang_prev) + 0.0*(error_ang_sum))
        #for i in range()
        rospy.loginfo("DMIN %s",d_min)
        #if d_min<1:
        #    msg.linear.x = 0
        #    if msg.angular.z<0 or f.0:
        #        msg.angular.z+=-0.5
        #        f+=1
        #    if msg.angular.z>0 or f ==2:
        #        f+=2
        #        msg.angular.z+=0.5
        #else:
        #    f = 0
        msg.linear.x = 2*error_dist + 0.5*(error_dist-error_dist_prev)
        

        rospy.loginfo("ERROR %s %s %s %s %s %s",theta,error_ang, error_dist, msg.angular.z, msg.linear.x, goal2)
        if msg.angular.z>1:
            msg.angular.z = 1
        elif msg.angular.z<-1:
            msg.angular.z = -1
        if msg.linear.x>1:
            msg.linear.x = 1
        elif msg.linear.x<0:
            msg.linear.x = 0

        error_ang_prev = error_ang
        error_ang_sum+=error_ang
        error_dist_prev = error_dist

        if abs(error_ang)<0.05*np.pi:
            error_ang_sum = 0
    #
    #for i in range(mode):
    #    if mode[i][0] == 0:
    #        if sin(atan2((mode[i][2]-y),(mode[i][1]-x)))>0.3 and sin(atan2((mode[i][4]-y),(mode[i][3]-x)))>0.3:
    #           pass:
    #        elif sin(atan2((mode[i][2]-y),(mode[i][1]-x)))>0.3 and 

    
    #rospy.loginfo("IN, %s, %s", r, len(r))   
    
    pub.publish(msg)
    
    rate = rospy.Rate(10) 
    flag += 1
    

def listener():
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, transform)
    rospy.Subscriber("/laser_scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass