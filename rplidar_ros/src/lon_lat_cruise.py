#!/usr/bin/env python
import rospy
import numpy as np
import threading
from std_msgs.msg import Float64MultiArray, Int16, UInt16, UInt16MultiArray, String

import requests
import json
import time
import string

yaw_g = Int16()

lon = float()
lat = float()

last_set_lon_int = np.zeros((1,1),np.int32)
last_set_lat_int = np.zeros((1,1),np.int32)
set_lon = float(0)
set_lat = float(0)
last_error = float(0) 
error_sum = float(0)

v=int(0)

#speed = 800
#dutt_M = 4500
Cruise_Speed_L = 4500
Cruise_Speed_R = 4500

flag_yaw = 0
flag_lon_lat = 0
flag_set = 0


def get_set_msg():
    global set_lon,set_lat,last_set_lon_int,last_set_lat_int,v,flag_set
    global Cruise_Speed_L,Cruise_Speed_R

    requests.adapters.DEFAULT_RETRIES = 5
    #url = 'http://47.100.92.173:10000/check'
    url = 'http://47.100.92.173:10000/allMsgCheck'
    #time_temp = None
    data = {
        "id":"wrc",
        'msg':'msg'
    }
    #try:
    req = requests.post(url=url,data=data,timeout=5).text
        #req_time = json.loads(req)['time']
    order = json.loads(req)['msg']
        #if(time_temp != req_time):
    l = int('0x'+order[0:2],0)
    #print(req)
    print(order)
    if(len(order)== 16*l+2):
        set_lon_int = np.zeros((l),int)
        set_lat_int = np.zeros((l),int)
        for i in range(l):
            set_lat_int[i] = int('0x'+order[i*16+2:i*16+10],0)
            set_lon_int[i] = int('0x'+order[i*16+10:i*16+18],0)
        if(np.sum(set_lon_int)!=np.sum(last_set_lon_int) and np.sum(set_lat_int)!=np.sum(last_set_lat_int)):
            v=0
            print(set_lon_int)
            print(last_set_lon_int)
        print("set lon:"+str(set_lon_int))
        print("set lat:"+str(set_lat_int))
        last_set_lon_int = set_lon_int
        last_set_lat_int = set_lat_int
        set_lon = [x/1000000.0 for x in set_lon_int] 
        set_lat = [x/1000000.0 for x in set_lat_int]
        flag_set =1
            #time_temp =req_time

    #except Exception:
    #    print("halt")
    #    Cruise_Speed_L = 4500
    #    Cruise_Speed_R = 4500
    #    flag_set = 0

def angleFromCoordinate(lat1,long1,lat2,long2):
    dlon= float(long2-long1)
    dlat= float(lat2-lat1)
    lat_r=(lat1+lat2)/360*np.pi
	  
    y =np.cos(lat_r)*dlon
    x = dlat

    brng = np.arctan2(y,x) 

    brng = (brng/np.pi)*180  #-180~180

    if(brng < 0):	
        brng =brng+360
    print("angle:"+str(brng))
    return float(brng)

def Yaw_Control(lat1,lon1,lat2,lon2):
    global last_error,yaw_g,error_sum
    u_temp = float(0)
    error = float(0)

    error = angleFromCoordinate(lat1,lon1,lat2,lon2) - yaw_g
    
    while(error < -180):
		error = error + 360
    while(error > 180):
		error = error - 360

    if(error > -90 and error < 90): 
		error_sum = error_sum + 0.1*error

    if(error_sum > 200):
		error_sum = 200
    if(error_sum < -200):
		error_sum = -200
    
    u_temp = 15 * error #+ 40*(last_error - error)
    
    last_error = error

    if(error > -1 and error < 1): 
		#u_temp = 0
		error_sum = 0

    return int(u_temp)

def Xunhang_Control():
    global lat,lon,set_lat,set_lon,v,Cruise_Speed_L,Cruise_Speed_R
    p=len(set_lat)
    print(set_lat)
    print(v)
    if(v>=p):
        v=0
    a=lat		
    b=lon								
    c=set_lat[v]
    d=set_lon[v]
    print(a)
    print(b)
    print(c)
    print(d)
    u=Yaw_Control(a,b,c,d)
    if(abs(u) > 400):
        fspeed = 0
    else:
        fspeed = 900
    Cruise_Speed_L = 4500+fspeed+u 	
    Cruise_Speed_R = 4500+fspeed-u 

    if(Cruise_Speed_L>6000):
        Cruise_Speed_L=6000
    if(Cruise_Speed_L<3000):
        Cruise_Speed_L=3000

    if(Cruise_Speed_R>6000):
        Cruise_Speed_R=6000
    if(Cruise_Speed_R<3000):
        Cruise_Speed_R=3000

    if(np.abs(int(a*1000000)-int(c*1000000))<=30 and abs(int(b*1000000)-int(d*1000000))<=40):
        v=v+1
    print(v)


def callback_yaw(yaw):
    global yaw_g,flag_yaw
    yaw_g = yaw.data
    flag_yaw = 1

def callback_lon_lat(lon_lat):
    global lon,lat,flag_lon_lat
    lat = lon_lat.data[0]
    lon = lon_lat.data[1]
    flag_lon_lat = 1

def callback_set_lon_lat(set_lon_lat):
    if(set_lon_lat.data == '1'):
        get_set_msg()


if __name__ == '__main__':
    rospy.init_node('cruise', anonymous=True)
    speed_pub = rospy.Publisher('cruise_speed',UInt16MultiArray, queue_size=1)
    cruise_speed = UInt16MultiArray()  

    rospy.Subscriber('yaw',Int16,callback_yaw)
    rospy.Subscriber('GPS',Float64MultiArray,callback_lon_lat)
    rospy.Subscriber('state',String,callback_set_lon_lat)

    while not rospy.is_shutdown():
        if(flag_yaw == 1 and flag_lon_lat == 1):
            flag_yaw =0
            flag_lon_lat = 0
            if(flag_set == 1):
                if len(set_lat)>0 :
                    Xunhang_Control()
            
            cruise_speed.data=[(Cruise_Speed_L+Cruise_Speed_R - 6000)//30,(Cruise_Speed_L - Cruise_Speed_R + 3000)//30]
            speed_pub.publish(cruise_speed)
            print("command:"+str(cruise_speed.data))
