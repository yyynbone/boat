#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import time, json,threading
import paho.mqtt.client as mqtt
from boat_msgs.msg import BoatGps,BoatImu,BoatStatus

gps_data, imu_data, status_data = {}, {}, {}
gps_flag, imu_flag, status_flag = 0, 0, 0

def callback_gps(gps):
    global gps_data, gps_flag

    heading = gps.heading
    heading += 90
    if heading >= 360:
        heading -= 360
    gps_data = {
        'lat': int(gps.latitude * 1000000)/1000000.0,
        'lon': int(gps.longitude * 1000000)/1000000.0,
        'yaw': int(heading*10)/10.0,
        'QF' : gps.QF,
        'sat': gps.number_satellites
    }
    gps_flag = 1

def callback_imu(imu):
    global imu_data, imu_flag
    imu_yaw = imu.euler_angle.z
    if imu_yaw > 0:
        imu_yaw = imu_yaw
    else:
        imu_yaw = 360 + imu_yaw
    imu_data = {
        'yaw'  : int(imu_yaw*10) / 10.0,
        'ang_v': int(imu.angular_velocity.z*10) / 10.0
    }
    imu_flag = 1

def callback_status(status):
    global status_data, status_flag
    status_data = {
        'motor_l': int(status.motor_left),
        'motor_R': int(status.motor_right),
        'voltage': int(status.voltage*10)/10.0,
        'light_l': int(status.light_left),
        'light_r': int(status.light_right),
        'imu_chk': int(status.health_imu),
        'gps_chk': int(status.health_gps)
    }
    status_flag = 1

def on_connect(client, userdata, flags, rc):
    print("Connected with result code: " + str(rc))

if __name__ == '__main__':
    rospy.init_node('info2net_mqtt', anonymous=True)

    rospy.Subscriber("boatimu", BoatImu, callback_imu)
    rospy.Subscriber("boatgps", BoatGps, callback_gps)
    rospy.Subscriber("boatstatus", BoatStatus, callback_status)

    client = mqtt.Client()
    client.on_connect = on_connect

    while not rospy.is_shutdown():
        #connect
        # try:
        #     client.connect('47.100.92.173', 11883, 60)
        #     print("pub_mqtt_connect")
        # except Exception:
        #     print("timeout,try again...")
        #     time.sleep(1)
        #     continue
        #publish
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                #client.connect('47.100.92.173', 11883, 60)
                client.connect('47.110.250.141', 7200, 3600)
                if gps_flag == 1:
                    gps_flag = 0
                    info = json.dumps(gps_data)
                    client.publish('as/wrc/under/gps',info, qos=0)
                    time.sleep(0.05)

                if imu_flag == 1:
                    imu_flag = 0
                    info = json.dumps(imu_data)
                    client.publish('as/wrc/under/imu',info, qos=0)
                    time.sleep(0.05)

                if status_flag == 1:
                    status_flag =0
                    info = json.dumps(status_data)
                    client.publish('as/wrc/under/status',info, qos=0)
                    time.sleep(0.05)
                client.disconnect()
                rate.sleep()
            except Exception:
                rate.sleep()
                break
