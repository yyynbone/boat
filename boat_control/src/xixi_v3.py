#!/usr/bin/python3
import matplotlib.pyplot as plt
import numpy as np
from copy import deepcopy
from scipy import interpolate
from scipy.interpolate import interp1d
import math
import networkx as nx
from mysql import get_graph
import time
import os
import paho.mqtt.client as mqtt
import threading, json
from boat_msgs.msg import BoatCmd,BoatGpsPoint,BoatGpsPoints
from std_msgs.msg import Float64MultiArray,Float32
import rospy
from rospy.numpy_msg import numpy_msg
def GpsPointToArray(point):
    return {
        'lon': int(point.longitude*1000000)/1000000.0,
        'lat': int(point.latitude*1000000)/1000000.0
    }

def GpsPointsToArray(points):
    result = []
    for i in points.points:
        result.append(GpsPointToArray(i))
    return result

def millerToXY(lon, lat):
    L = 6381372.0 * math.pi * 2.0  
    W = L  
    H = L / 2.0  
    mill = 2.3
    x = float(lon) * math.pi / 180.0  
    y = float(lat) * math.pi / 180.0
    y = 1.25 * math.log(math.tan(0.25 * math.pi + 0.4 * y)) 
    x = (W / 2) + (W / (2 * math.pi)) * x
    y = (H / 2) - (H / (2 * mill)) * y
    return int(round(x)), int(round(y))

def nearest_node(id_gps,node):
    node_x, node_y = millerToXY(node[0], node[1])
    distance=[]
    for key,values in  id_gps.items():
        distance.append([key,(values[2]-node_x)**2+(values[3]-node_y)**2])
    distance.sort(key=lambda x: x[1])
    return int(distance[0][0])

def find_way(start,goal,G,id_gps):
    #global start,goal
    #final_way=[]
    path_to_navigation=BoatGpsPoints()
    # print(start,goal )
    # find the nearest node and the path
    nearest_list_start= nearest_node(id_gps,start) # gps
    nearest_list_goal= nearest_node(id_gps,goal)
    # get way
    #print(nearest_node_start,nearest_node_goal)
    way=nx.shortest_path(G, source=nearest_list_start, target=nearest_list_goal, weight=True, method='dijkstra')
    print(way)
    # show img and all nodes
    for i in range (len(way)): 
        node=BoatGpsPoint()
        node.longitude=float(id_gps[str(way[i])][0])
        node.latitude=float(id_gps[str(way[i])][1])
        path_to_navigation.points.append(node)
        print(str(way[i]),node)
    return path_to_navigation
    #print(time.time())

def on_connect1(client, userdata, flags, rc):
    print("Connected with result code: " + str(rc))

def on_disconnect1(client, userdata, flags, rc):
    print("disConnected with result code: " + str(rc))
    
def on_message1(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))
state = 0

def check(point,pointnow,i):
    node_x0,node_y0 = millerToXY(point[0].longitude,point[0].latitude)
    node_x1,node_y1 = millerToXY(point[1].longitude,point[1].latitude)
    start_x,start_y = millerToXY(pointnow[i].longitude,pointnow[i].latitude)
    node_xf0,node_yf0 = millerToXY(point[-1].longitude,point[-1].latitude)
    node_xf1,node_yf1 = millerToXY(point[-2].longitude,point[-2].latitude)
    end_x,end_y = millerToXY(pointnow[i+1].longitude,pointnow[i+1].latitude)
    if ((node_x1-start_x)**2+(node_y1-start_y)**2) < ((node_x1-node_x0)**2+(node_y1-node_y0)**2):
        point.pop(0)
    if ((node_xf1-end_x)**2+(node_yf1-end_y)**2) < ((node_xf1-node_xf0)**2+(node_yf1-node_yf0)**2):
        point.pop(-1)
    return point

def planning_callback(msg):
    global G, id_gps,cmd

    path=BoatGpsPoints()
    path_to_navigation=BoatGpsPoints()

    path.points.append(msg.points[0])
    #print(0)
    for i in range(len(msg.points)-1):
        start = [msg.points[i].longitude,msg.points[i].latitude]
        goal = [msg.points[i+1].longitude,msg.points[i+1].latitude]
        path_to_navigation=find_way(start,goal,G, id_gps)
        #print(1)
        #print(path_to_navigation.points)

        path_to_navigation.points = check(path_to_navigation.points,msg.points,i)
	    
        for point in path_to_navigation.points:
            path.points.append(point)
        path.points.append(msg.points[i+1])
    #print(mess)
    #print(msg.topic + " " + str(mess))
    client1 = mqtt.Client()
    #client1.connect('47.100.92.173', 11883, 600)
    client1.connect('47.110.250.141', 7200, 3600)
    print("path is :",path)
    info = json.dumps(GpsPointsToArray(path))
    pub1.publish(path)
    client1.publish('as/wrc/under/map',info, qos=0)
    client1.disconnect()
    
if __name__=="__main__":
   
    G, id_gps = get_graph()
    rospy.init_node('planning_nodes', anonymous=True)
    rospy.Subscriber('set_gps',BoatGpsPoints,planning_callback )
    pub1 = rospy.Publisher("nodes",BoatGpsPoints, queue_size=1)
    rospy.spin()
