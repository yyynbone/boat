#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
import cv2
import matplotlib.pyplot as plt
import os
import sys
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import Float64MultiArray,Float32,String
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg

num_pic=0
data_list=[]
def img_callback(msg):
    global img,index,img_flag
    cv= CvBridge()
    img = cv.imgmsg_to_cv2(msg, "mono8")

    img_flag = 1
    #print("1")
    index=index+1
    
def origin_img_callback(msg):
    global origin_img
    cv= CvBridge()
    origin_img = cv.imgmsg_to_cv2(msg, "bgr8")

def boat_callback(msg):
	global data_list
	data_list = msg.data
	
    
def get_direction(img): 
    global num_pic,origin_img,data_list  
    num_pic=num_pic+1    
    cv_img=img
    cv_origin_img=cv2.resize(origin_img, (640,400), interpolation=cv2.INTER_CUBIC)
    pic_path="/home/hsm/lz_pic/"+str(num_pic)+".jpg"
    cv2.imwrite(pic_path,cv_img)
    #print("100")
    cv_img=cv2.resize(cv_img, (640,400), interpolation=cv2.INTER_CUBIC)
    img1 = np.zeros((400,640),dtype="uint8")
    for i in range(int(len(data_list)/4)):
        for col in range(int(data_list[i*4]),int(data_list[i*4+2])):
            for row in range(int(data_list[i*4+1]),int(data_list[i*4+3])):
                img1[row][col] = 255
    cv_img = cv_img + img1 
    #print(cv_img.shape)
    ret, cv_img = cv2.threshold(cv_img, 1, 255, 0)
    contours= cv2.findContours(cv_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[1]
    if contours:
        high  = [cv_img.shape[1]/2,0] 
    else:
        contours_=max(contours, key=len)
        contours_=np.squeeze(contours_)
        contours_=sorted(contours_,key = lambda x:x[1])
        high=contours_[0]
    x = [[cv_img.shape[1]/2, high[0]]] 
    y = [[cv_img.shape[0], high[1]]]
    if abs(high[0])+abs(high[1])!=0:        
        cv2.line(cv_origin_img, (x[0][0],y[0][0]),(x[0][1],y[0][1]),(255,255,0),10)
        pic_path="/home/hsm/lz_pic/"+str(num_pic)+"s.jpg"
        cv2.imwrite(pic_path,cv_origin_img)
        theta=90.0-math.atan2(cv_img.shape[0]-high[1],high[0]-cv_img.shape[1]/2.5)*180/math.pi
        #print(theta)
        return theta
    else:
        return 0
    
rospy.init_node('hedao', anonymous=True)
pub1 = rospy.Publisher("hedao_direction",Float32, queue_size=1)
rospy.Subscriber("image/mask", Image, img_callback)
rospy.Subscriber("/mynteye/left_rect/image_rect", Image, origin_img_callback)
rospy.Subscriber("/boat_pred/data", Float64MultiArray, boat_callback)

direction=Float32()
img_flag=0
index=0
while not rospy.is_shutdown():
    if(img_flag==1):
        img_flag = 0
        direction.data= get_direction(img)
        pub1.publish(direction)
        #print("hedao direction is :",direction)