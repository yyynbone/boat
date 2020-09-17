import rospy
from std_msgs.msg import String
from boat_msgs.msg import BoatCmd,BoatGpsPoint,BoatGpsPoints

#import paho.mqtt.client as mqtt
#import json, time, string, signal
if __name__ == '__main__':
    #topic = msg.topic
    data = {'point[0]':0,'point[1]':1,'point[2]':2,'point[3]':3,'point[4]':4,'point[5]':5}
    #cmd = BoatCmd()
    #print(topic)
    print(data)
    setl=[0]*3
    #print("map")
    #gps=BoatGpsPoint()
    set_gps = BoatGpsPoints()
    print(len(data)/2)
    for i in range (int(len(data)/2)):
        #print(i)
        gps=BoatGpsPoint()
        print(gps)
        gps.longitude = float(data['point[%d]'% (i*2)])
        gps.latitude = float(data['point[%d]'% (i*2+1)])
        print(gps)
        print("##################")
        set_gps.points.append(gps)
        print(set_gps)
        setl[i]=gps
        print("-------")
        print(setl)
