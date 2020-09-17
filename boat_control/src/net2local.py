#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from boat_msgs.msg import BoatCmd

import requests
import json

import time
import string

requests.adapters.DEFAULT_RETRIES = 5

def listen_and_publish():
    #i = 0
    url = 'http://47.100.92.173:10000/check' 
    data = {
        'id':'wrc'
    }
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        try:
            req = requests.post(url = url,data = data, timeout = 5).text
            req_time = json.loads(req)['time']
            rev = json.loads(req)['order']
            print(rev)
            #i+=1
            #if(i==1):
            #    time_temp=req_time
            #    continue
            #if(i>=2):
                #if(time_temp != req_time):
            time_temp=req_time
            isAutoPilot = int(rev[1])
            speed = int('0x'+rev[2:4],0)
            angular_speed = int('0x'+rev[4:6],0)
            cmd.cmd = BoatCmd.SPEED
            cmd.data_float32 = [speed-100.0,(angular_speed-100.0)/10.0]
            cmd.data_int = []
            pub_cmd.publish(cmd)
            pub_state.publish(str(isAutoPilot))
            #print(1)
            rate.sleep()

        except Exception:
            cmd.cmd = BoatCmd.STOP
            cmd.data_float32 = []
            cmd.data_int = []
            pub_cmd.publish(cmd)
            pub_state.publish(str(0))
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('net2local',anonymous=True)
    pub_cmd = rospy.Publisher('manual_cmd', BoatCmd, queue_size = 10)
    pub_state = rospy.Publisher('auto_state', String, queue_size = 1)
    cmd = BoatCmd()
    listen_and_publish()
