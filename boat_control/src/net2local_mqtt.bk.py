#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from boat_msgs.msg import BoatCmd,BoatGpsPoint,BoatGpsPoints

import paho.mqtt.client as mqtt
import json, time, string, signal

# def GpsPointToArray(point):
#     return {
#         'lon': int(point.longitude*1000000)/1000000.0,
#         'lat': int(point.latitude*1000000)/1000000.0
#     }

# def GpsPointsToArray(points):
#     result = []
#     for i in points.points:
#         result.append(GpsPointToArray(i))
#     return result

def on_message(client, userdata, msg):
    topic = msg.topic
    data = json.loads(msg.payload)
    cmd = BoatCmd()
    print(topic)
    print(data)
    if topic == 'as/wrc/upper/move':
        cmd.cmd = BoatCmd.SPEED
        cmd.data_float32 = [float(data['fbValue']), float(data['lrValue']/10.0)]
        pub_manual_cmd.publish(cmd)
        pub_state.publish(str(data['pilotMode']))
    elif topic == 'as/wrc/upper/operate':
        cmd.cmd = BoatCmd.LIGHT
        cmd.data_int = [int(data['resetLight']), int(data['leftLight']), int(data['rightLight']), int(data['horn'])]
        pub_cmd.publish(cmd)
    elif topic == 'as/wrc/upper/map':
        start_gps, end_gps = BoatGpsPoint(), BoatGpsPoint()
        set_gps = BoatGpsPoints()
        start_gps.latitude = float(data['startLat'])
        start_gps.longitude = float(data['startLon'])
        end_gps.latitude = float(data['endLat'])
        end_gps.longitude = float(data['endLon'])        
        set_gps.points = [start_gps,end_gps]

        # info = json.dumps(GpsPointsToArray(set_gps))
        # client.publish('as/wrc/under/map',info, qos=0)
        print("gps:  ",set_gps)
        pub_gps.publish(set_gps)
        pub_state.publish(str(1))

def on_connect(client, userdata, flags, rc):
    print("Connected with result code: " + str(rc))

def on_disconnect(client, userdata, flags, rc):
    print("disConnected with result code: " + str(rc))

def client_connected():
    while True:
        time.sleep(1)
        try:
            #client.connect('47.100.92.173', 11883, 60) # 600 is keepalive
            client.connect('47.110.250.141', 7200, 3600)
            client.subscribe('as/wrc/upper/#', qos=0)
            client.loop_forever() # keeplive
            break
        except Exception:
            cmd = BoatCmd()
            cmd.cmd = BoatCmd.STOP
            cmd.data_float32 = []
            cmd.data_int = []
            pub_manual_cmd.publish(cmd)
            pub_state.publish(str(0))
            print("timeout,try again...")
            

def my_handler(signum,frame):
    client.disconnect()

if __name__ == '__main__':
    rospy.init_node('net2local_mqtt',anonymous=True)
    pub_manual_cmd = rospy.Publisher('manual_cmd', BoatCmd, queue_size = 10)
    pub_cmd =  rospy.Publisher('boatcmd', BoatCmd, queue_size = 10)
    pub_state = rospy.Publisher('auto_state', String, queue_size = 1)
    pub_gps = rospy.Publisher('set_gps',BoatGpsPoints, queue_size= 2)
    cmd = BoatCmd()

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    signal.signal(signal.SIGINT, my_handler)

    client_connected()
