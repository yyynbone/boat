#!/usr/bin/env python
import rospy
import numpy as np
from visualization_msgs.msg import Marker
import cv2
import matplotlib.pyplot as plt 
import time
import math

img = np.zeros((500, 500), np.uint8)

def callback(lines):
    global img
    img = img*0
    size = len(lines.points) / 8
    pps = []
    for i in range(size):
        ps = []
        ps.append([int(lines.points[8*i].x*10)+250,int(lines.points[8*i].y*10)+250])
        ps.append([int(lines.points[8*i+2].x*10)+250,int(lines.points[8*i+2].y*10)+250])
        ps.append([int(lines.points[8*i+4].x*10)+250,int(lines.points[8*i+4].y*10)+250])
        ps.append([int(lines.points[8*i+6].x*10)+250,int(lines.points[8*i+6].y*10)+250])
        pps.append(np.array(ps))
    cv2.fillPoly(img, pps, 255)

if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    rospy.Subscriber('lines', Marker,callback)
    while not rospy.is_shutdown():
        cv2.imshow("Test",img)
        cv2.waitKey(100)
    #plt.imshow(img)
    #plt.show()