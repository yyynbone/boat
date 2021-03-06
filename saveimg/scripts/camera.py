#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import os

# Instantiate CvBridge
bridge = CvBridge()
global i
i = 0
def image_callback(msg):
    global i
    print("Received an image!")
    #try:
        # Convert your ROS Image message to OpenCV2
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgra8")
    #except CvBridgeError, e:
    #    print(e)
    #else:
        # Save your OpenCV2 image as a jpeg
    i = i + 1
    if(i%10==0):
        print("Capture Image:"+str(i/10)+',jpeg')
        cv2.imwrite('/home/hsm/image/'+str(i/10)+'.jpeg', cv2_img)

def depth_callback(msg):
    global i
    print("Received depth")
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgra8")
    if(i%10==0):
        print("Capture Image:"+str(i/10)+'depth'+',jpeg')
        cv2.imwrite('/home/hsm/image/'+str(i/10)+'depth'+'.jpeg', cv2_img)


def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/mynteye/left_rect/image_rect"
    #depth_topic = "/depth"
    # Set up your subscriber and define its callback
    path="/home/hsm/image/"
    if os.path.exists(path):
        pass
    else:
        os.mkdir(path)
    rospy.Subscriber(image_topic, Image, image_callback)
    #rospy.Subscriber(depth_topic, Image, depth_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
