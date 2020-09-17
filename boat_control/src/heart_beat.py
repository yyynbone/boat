#!/usr/bin/env python
import rospy
from boat_msgs.msg import BoatHeartBeat

if __name__ == '__main__':
    rospy.init_node('heartbeat',anonymous=True)
    pub_heartbeat = rospy.Publisher('boatheartbeat', BoatHeartBeat, queue_size = 10)
    heartbeat = BoatHeartBeat()
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        pub_heartbeat.publish(heartbeat)
        rate.sleep()