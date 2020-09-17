#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
from visualization_msgs.msg import Marker
import numpy as np
import cv2 as cv
from APF import APFplanning,univector
import math
flag = 0
kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
img = np.ones((500, 500), np.uint8)*255

def callback(lines):
    global img,flag,kernel,opening
    img = img*0+255
    size = len(lines.points) / 8
    pps = []
    for i in range(size):
        ps = []
        ps.append([int(lines.points[8*i].x*10)+250,int(lines.points[8*i].y*10)+250])
        ps.append([int(lines.points[8*i+2].x*10)+250,int(lines.points[8*i+2].y*10)+250])
        ps.append([int(lines.points[8*i+4].x*10)+250,int(lines.points[8*i+4].y*10)+250])
        ps.append([int(lines.points[8*i+6].x*10)+250,int(lines.points[8*i+6].y*10)+250])
        pps.append(np.array(ps))
    opening = cv.fillPoly(img, pps, 0)    
    #opening = cv.erode(img, kernel) 
    #img = cv.dilate(img, kernel) 
    #opening = cv.morphologyEx(img, cv.MORPH_OPEN, kernel)
    flag = 1
    #print(2)

if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    rospy.Subscriber('lines', Marker,callback)
    direct = []
    while not rospy.is_shutdown():
        if(flag == 1):
            # print(1)
            flag = 0
            point_size = 1
            point_color = (0, 0, 0)  
            thickness = 4  
            pic = opening#img
            start = [250, 250]
            goal = [300, 250]
            pic = np.where(pic > 128, 255, 0)
            w = APFplanning(start, goal, 50, 1, 3, pic, 30, 10, 3)
            #path=(int(math.cos(w)*20.0)+250,int(math.sin(w)*20.0)+250)
            
            #direct.append(univector([path[0]-start[0],path[1]-start[1]]))
            print(path)
            #cv.line(opening,(250,250),path,point_color)
            #[cv.circle(img, (int(path[1][i]), int(path[0][i])), point_size, point_color, thickness) for i in range(len(path[0]))]
            #cv.circle(img, (250,10), point_size, point_color, thickness)
            #cv.imshow("Test",opening)
            #cv.waitKey(100)