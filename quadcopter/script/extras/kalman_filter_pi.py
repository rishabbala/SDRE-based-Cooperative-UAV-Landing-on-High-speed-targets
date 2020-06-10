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
#from filterpy.kalman import KalmanFilter

global xval,yval,xsig,ysig,sig1,sig2,wval,wsig,x_velocity,y_velocity,alt,now_time,last_time,center_pred_x,center_pred_y,center_x,center_y,flag_tracking,notFoundCount,flag_detected,xt_image,yt_image,width,height,P
global flag_first_detection,flag_started_vis,flag_imageshow,flag_apriltag
flag_started_vis=0
flag_first_detection=0
flag_apriltag=0
xval=0
yval=0
xsig=1000
ysig=1000
sig1=1
sig2=1
wsig=1000
wval=1
x_velocity=0
y_velocity=0
alt=0
now_time=0;last_time=0;center_pred_x=[];center_pred_y=[];center_x=[];center_y=[];xt_image=320;yt_image=240;width=0;height=0;flag_detected=-1
flag_tracking=0;notFoundCount=0
flag_imageshow=1
def Receivealt(data):
    global alt
    alt=data.data
    return alt

def segment_colour(frame):
    hsv_roi =  cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_1 = cv2.inRange(hsv_roi, np.array([160, 160,10]), np.array([190,255,255]))
    #mask_1 = cv2.inRange(hsv_roi, np.array([160.,120.,40.]), np.array([190.,255.,255.]))
    ycr_roi=cv2.cvtColor(frame,cv2.COLOR_BGR2YCrCb)
    mask_2=cv2.inRange(ycr_roi, np.array((0.,165.,0.)), np.array((255.,255.,255.)))
    mask= mask_1 | mask_2
    #mask_2=cv2.inRange(ycr_roi, np.array((30.,165.,0.)), np.array((255.,255.,255.)))
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
    r=(0,0,2,2)
    if len(contours) > 0:
        r = cv2.boundingRect(contours[cont_index])
        #print "here"

    return r


def receiveimage(data):
    global x_velocity,y_velocity,roll,pitch,yaw,out,xval,yval,xsig,ysig,sig1,sig2,wval,wsig,x_velocity,y_velocity,alt,a_orig,cvFrame,flag_started_vis,a,b,c,d,flag_tracking,xt_image,yt_image,width,height,flag_detected,flag_imageshow,flag_apriltag
    bridge=CvBridge()
    orig_img = bridge.imgmsg_to_cv2(data,"passthrough")
    cvFrame = orig_img ##cv2.flip(orig_img, 0)
    mask_red=segment_colour(cvFrame)
    loct=find_blob(mask_red)
    x,y,w,h=loct
    info = TargetInfo()
    contour = Contour()
    center = Point32()
    flag_started_vis_prev=flag_started_vis
    #print a,b,c,d,flag_tracking
    if(w*h<10):
        info.detected=0
        center.x=-1
        center.y=-1
        contour.width=-1
        contour.height=-1
        flag_detected=info.detected
    else:
        info.detected = 1
        center.x = x + (w / 2)
        center.y = y + (h / 2)
        contour.width = w
        contour.height = h
        xt_image=center.x
        yt_image=center.y
        width=contour.width
        height=contour.height
        flag_detected=info.detected
        if(flag_started_vis==0):
            flag_started_vis=1
        #cv2.rectangle(cvFrame, (x, y), (x + w, y + h), 255, 2)
        #cv2.circle(cvFrame, (int(center.x), int(center.y)), 3, (0, 110, 255), -1)
    if(flag_tracking==1):
        cv2.rectangle(cvFrame, (a-c/2,b-d/2), (a+c/2, b+d/2), 110, 2)
        cv2.circle(cvFrame, (int(a), int(b)), 3, (110, 0, 255), -1)
    

    
    '''
    if(info.detected==1):
        fp=(0.8/contour.width)*640
        fov=2*math.degrees(math.atan2((fp/2),alt))
        print "center_x:",center.x,"center_y:",center.y,"detected:",info.detected,"width:",contour.width,"height:",contour.height,"alt:",alt,"fp:",fp,"fov:",fov,"fp:",(fp/1.5),"x_velocity:",x_velocity,"y_velocity:",y_velocity,"cmd_x:",a_orig[0],"cmd_y:",a_orig[1],"cmd_z:",a_orig[2]
    '''
    
    
    flag_first_detection=flag_started_vis-flag_started_vis_prev

    
    if(flag_imageshow==1):
        cv2.imshow('img',cvFrame)
        cv2.waitKey(1)
    k = cv2.waitKey(5) & 0xff
    if k == 27:
        flag_imageshow=0
        cv2.destroyAllWindows()

    
    #print a,b,c,d,flag_tracking
    return flag_detected,xt_image,yt_image,width,height,flag_apriltag

'''
def ReceiveVisionMeas(data):
    global xt_image,yt_image,zt,flag_detected,tar_dist,width,height
    print flag_detected
    flag_detected=data.detected
    xt_image=data.contour.center.x
    yt_image=data.contour.center.y
    width=data.contour.width
    height=data.contour.height
    zt=0
    return xt_image,yt_image,zt,flag_detected,width,height
'''


def kalmanEst(event):
    global xval,yval,xsig,ysig,sig1,sig2,wval,wsig,x_velocity,y_velocity,alt,X,F,H,Z,P,Process_noise_cov,Meas_noise_cov,now_time,last_time,flag_tracking,notFoundCount,flag_detected,xt_image,yt_image,width,height,flag_apriltag
    global a,b,c,d
    #print flag_detected
    #rate = rospy.Rate(1)
    now_time=rospy.get_time()
    #print flag_tracking
    dt=0.033
    F[0][0]=1;F[0][2]=dt;F[1][1]=1;F[1][3]=dt;F[2][2]=1;F[3][3]=1;F[4][4]=1;F[5][5]=1
    rate = rospy.Rate(25)
    while not rospy.is_shutdown():
        if(flag_first_detection==1):#First Detection
            notFoundCount=0
            #print "in_first"
            flag_tracking=0
            X=np.zeros((6,1))
            X[0]=center.x;X[1]=center.y;X[2]=0;X[3]=0;X[4]=contour.width;X[5]=contour.height
            P=np.zeros((6,6))
            P[0][0]= 1;P[1][1]= 1;P[2][2]= 1;P[3][3]= 1;P[4][4]= 1;P[5][5]= 1
    
    
        if(flag_tracking==1):
            X=np.dot(F,X)
            if(X[0][0]>=640):
                X[0][0]=640
            if(X[1][0]>=480):
                X[1][0]=480
            if(X[0][0]<=0):
                X[0][0]=0
            if(X[1][0]<=0):
                X[1][0]=0

            P=np.dot(np.dot(F,P),F.T) + Process_noise_cov
            #P = (F * P *F.T) + Process_noise_cov
            #print "in_pred"
        
        
        #print flag_detected,xt_image,yt_image,width,height
        if(flag_detected==1):
            flag_tracking=1
            notFoundCount=0
            #print "in detected 1"
        elif(flag_detected==0):
            if(flag_tracking==1):
                notFoundCount=notFoundCount+1
            if(notFoundCount>=9000):
                flag_tracking=0
        



        
        
        
        if(flag_detected==1):
            Z[0]=xt_image;Z[1]=yt_image;Z[2]=width;Z[3]=height
            Zpred = np.dot(H,X)
            Innov = Z-Zpred
            S = np.dot(np.dot(H,P),H.T) + Meas_noise_cov
            W = np.dot(np.dot(P,H.T),np.linalg.inv(S))
            X = X+ np.dot(W,Innov)
            P=P-np.dot(np.dot(W,H),P)
            
        last_time=now_time
        a=int(X[0][0])
        b=int(X[1][0])
        c=int(X[4][0])
        d=int(X[5][0])
        info = TargetInfo()
        contour_pred = Contour()
        center = Point32()
        #print "center_x:",a,"center_y:",b,"width:",c,"height:",d,"xdot:",X[2][0],"ydot:",X[3][0],"alt:",alt
        
    
        #cv2.rectangle(cvFrame, (x, y), (x + int(X[4][0]), y + int(X[5][0])), 255, 2)
        #cv2.circle(cvFrame, (int(X[0][0]), int(X[1][0])), 3, (0, 110, 255), -1)
        
        contour_pred.center.x = X[0][0]
        contour_pred.center.y=X[1][0]
        contour_pred.width=X[4][0]
        contour_pred.height=X[5][0]
        info.contour = contour_pred
        info.detected=flag_detected
        info.apriltag=flag_apriltag
        info.tracking=flag_tracking
        info_pub.publish(info)
        rate.sleep()
        #measurement.publish(measured_pub)
        
        #cv2.imshow('img',cvFrame)
        #cv2.waitKey(1)
        
        #contour_pub.publish(rosFrame)
        
        #k = cv2.waitKey(5) & 0xff
        #if k == 27:
        #    break
        #cv2.destroyAllWindows()
        

    

def identifier():
    rospy.init_node('identifier_estimate', anonymous=True)
    global info_pub,Process_noise_cov,Meas_noise_cov,F,H,X,Z,P
    global contour_pub, measurement
    global info_pub
    global contour_pub,raw_pub
    info_pub = rospy.Publisher('/landing_target_info_new', TargetInfo, queue_size=10)
    #rospy.Subscriber('landing_target_info', TargetInfo, ReceiveVisionMeas)
    rospy.Subscriber('/camera_on_quad/images', Image, receiveimage)
    rospy.Subscriber('/drone/mavros/global_position/rel_alt',Float64,Receivealt)
    dt=0.2
    F=np.zeros((6,6))
    H=np.zeros((4,6))
    F[0][0]=1;F[0][2]=dt;F[1][1]=1;F[1][3]=dt;F[2][2]=1;F[3][3]=1;F[4][4]=1;F[5][5]=1
    H[0][0]=1;H[1][1]=1;H[2][4]=1;H[3][5]=1
    X=np.zeros((6,1))
    Z=np.zeros((4,1))
    X[0][0]=320
    X[1][0]=240
    Z[0]=320
    Z[1]=240
    Process_noise_cov=np.zeros((6,6))
    Process_noise_cov[0][0]= 1e-6;
    Process_noise_cov[1][1]= 1e-6;
    Process_noise_cov[2][2]= 1e-9;
    Process_noise_cov[3][3]= 1e-9;
    Process_noise_cov[4][4]= 1e-5;
    Process_noise_cov[5][5]= 1e-5;
    Meas_noise_cov=np.zeros((4,4))
    Meas_noise_cov[0][0]= 1e-15;
    Meas_noise_cov[1][1]= 1e-15;
    Meas_noise_cov[2][2]= 1e-5;
    Meas_noise_cov[3][3]= 1e-5;
    P=np.zeros((6,6))
    P[0][0]= 320;P[1][1]= 240;P[2][2]= 1;P[3][3]= 1;P[4][4]= 1;P[5][5]= 1
    #callback()
    kalman_timer=rospy.Timer(rospy.Duration(33/1000.0),kalmanEst)
    rospy.spin()

if __name__ == '__main__':
    try:
        identifier()
    except rospy.ROSInterruptException:
        pass
                                                                                                                                  