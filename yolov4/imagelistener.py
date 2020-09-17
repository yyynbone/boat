#!/usr/bin/python3
#subscriber node
#edit by hzh 2020/09/10
import rospy
#from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import numpy as np
import cv2
def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s",data.data)
    print(data.data[:20])
    img = np.array(data.data)
    img = img.reshape(400,640,3) #此时图像为rgb格式，如果要用cv显示，需要转成bgr
    cv2.imshow("img",img)
    cv2.imwrite("./1.jpg",img)
    cv2.waitKey()
    
    
def listener():
    rospy.init_node("imagelistener", anonymous=True)
    rospy.Subscriber("/origin_data",Float64MultiArray,callback)

    #spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
