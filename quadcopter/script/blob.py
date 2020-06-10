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
from nav_msgs.msg import Odometry

rospy.init_node('identifier_estimate', anonymous=True)
pub = rospy.Publisher('/landing_target_info_new', TargetInfo, queue_size=1)
now = rospy.get_time()

global info, flag_imageshow, cvFrame
cvFrame = np.zeros((2000,2000,3), np.uint8)
info = TargetInfo()
flag_imageshow=1

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
    global info, flag_imageshow, cvFrame
    bridge=CvBridge()
    cvFrame = bridge.imgmsg_to_cv2(data,"passthrough")

def color_det(event):
    global info, flag_imageshow, cvFrame, now
    frame = cvFrame
    now = rospy.get_time()
    #cvFrame = frame
    #rate = rospy.Rate(20)

    bridge=CvBridge()
    mask_red = segment_colour(frame)
    x, y, radius = find_blob(mask_red)
    if(radius<5):
        info.detected=0
        info.center.x=-1
        info.center.y=-1
        info.radius=-1
    else:
        info.detected = 1
        info.center.x = x
        info.center.y = y
        info.radius = radius
        cv2.circle(frame, (int(x),int(y)), int(radius), (0, 255, 0), 2)
        cv2.circle(frame, (int(x), int(y)), 3, (110, 0, 255), -1)

    print(info)

    info.time = now
    #rospy.loginfo(info.time)
    pub.publish(info)
    #re = cv2.resize(frame, (500, 500), interpolation = cv2.INTER_AREA)

    #if flag_imageshow == 1:
    #    cv2.imshow('detection',re)
    #    cv2.waitKey(1)
    #k = cv2.waitKey(5) & 0xff
    #if k == 27:
    #    flag_imageshow = 0
    #    cv2.destroyAllWindows()
    #rate.sleep()


def listener():
    rospy.Subscriber('/camera_on_quad0/images0', Image, receiveimage)
    timer=rospy.Timer(rospy.Duration(20/1000.0),color_det)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
