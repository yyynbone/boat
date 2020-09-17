#!/usr/bin/env python
# -*- coding: utf-8 -*
import rospy
import numpy as np
import requests
import threading

from boat_msgs.msg import BoatGps,BoatImu,BoatStatus,BoatVel
from std_msgs.msg import String

requests.adapters.DEFAULT_RETRIES = 5
url = 'http://47.100.92.173:10000/gps'

lat_int = int()
lon_int = int()
motor_L = int()
motor_R = int()
gps_yaw = int()
imu_yaw = int()
voltage = int()
front_m = int()
behind_m = int()
check = int()
sat = int()
auto_state = int()

def int2hex2str(data,num):
    temp = int(data)
    result = str()
    for i in range(num):
        result = str(hex(int(temp%16))[2:3])+result
        temp = int(temp / 16)
    return result

def callback_state(state):
    global auto_state
    auto_state = int(state.data)

def callback_gps(gps):
    global lat_int,lon_int,gps_yaw,sat
    lat_int = int(gps.latitude * 1000000)
    lon_int = int(gps.longitude * 1000000)
    gps_yaw = int(gps.heading)
    gps_yaw = gps_yaw + 90
    if gps_yaw > 360:
        gps_yaw -= 360
    temp = int(gps.number_satellites)
    if temp > 15:
        temp = 15
    sat = int(gps.QF*16+temp)


def callback_imu(imu):
    global imu_yaw
    imu_yaw = imu.angular_velocity.z
    #if imu_yaw < 0:
    #    imu_yaw = -imu_yaw
    #else:
    #    imu_yaw = 360 - imu_yaw
    imu_yaw = int(100 + imu_yaw*10)
    if imu_yaw > 200:
        imu_yaw = 200
    if imu_yaw < 0:
        imu_yaw = 0


def callback_status(status):
    global motor_L,motor_R,voltage
    motor_L = int(100+status.motor_left)
    motor_R = int(100+status.motor_right)
    voltage = int(status.voltage * 10)
    
def callback_vel_info(vel_info):
    global imu_yaw
    imu_yaw = int(100 + vel_info.w *10)
    if imu_yaw > 200:
        imu_yaw = 200
    if imu_yaw < 0:
        imu_yaw = 0

if __name__ == '__main__':

    rospy.init_node('info2net', anonymous=True)

    rospy.Subscriber("auto_state",String,callback_state)
    rospy.Subscriber("boatimu",BoatImu,callback_imu)
    rospy.Subscriber("boatgps",BoatGps,callback_gps)
    rospy.Subscriber("boatstatus",BoatStatus,callback_status)
    #rospy.Subscriber("boatvel",BoatVel,callback_vel_info)
    
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():

        #str_send = int2hex2str(int('0xA5',0),2) + int2hex2str(int('0x5A',0),2)
        str_send = int2hex2str(lat_int,8) + int2hex2str(lon_int,8) + int2hex2str(motor_L,2) + int2hex2str(motor_R,2)
        str_send = str_send + int2hex2str(gps_yaw,4) + int2hex2str(imu_yaw,4) + int2hex2str(voltage,4) + int2hex2str(front_m,4) + int2hex2str(behind_m,4)
        str_send = str_send + int2hex2str(check,2) + int2hex2str(sat,2) + int2hex2str(auto_state,2)
        sum_check = 0
        for i in range(23):
            sum_check = sum_check + int(('0x'+str_send[i*2:i*2+2]),0)
        sum_check = int(sum_check % 256)
        str_send = int2hex2str(int('0xA5',0),2) + int2hex2str(int('0x5A',0),2) + str_send + int2hex2str(sum_check,2)
        try:
            data4gps = {
            "msg" : str_send,
            'id':'wrc'
            }
            req = requests.post(url = url , data=data4gps , timeout=5).text
            # print(str_send)
            rate.sleep()
        except Exception:
            rate.sleep()


