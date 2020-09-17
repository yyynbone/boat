#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from boat_msgs.msg import BoatCmd

auto_state = 0
temp_manual_state = 0

def callback_state(state):
    global auto_state
    auto_state = int(state.data)

def callback_manual(cmd):
    global auto_state, temp_manual_state
    if(auto_state == 0):
        pub_cmd.publish(cmd)
    elif(cmd.cmd == BoatCmd.STOP):
        temp_manual_state = 1
        pub_cmd.publish(cmd)
    elif(cmd.cmd == BoatCmd.SPEED):
        if(len(cmd.data_float32)==2):
            if(cmd.data_float32[0] != 0 or cmd.data_float32[1] != 0):
                temp_manual_state = 1
                pub_cmd.publish(cmd)
            else:
                temp_manual_state = 0 
        elif(len(cmd.data_int)==2):
            if(cmd.data_int[0] != 0 or cmd.data_int[1] != 0):
                temp_manual_state = 1
                pub_cmd.publish(cmd)
            else:
                 temp_manual_state =0
        else:
            temp_manual_state = 0
    else:
        temp_manual_state = 0

def callback_auto(cmd):
    global auto_state, temp_manual_state
    if(auto_state == 1 and temp_manual_state == 0):
        pub_cmd.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('cmd_speed_manage',anonymous=True)
    rospy.Subscriber("manual_cmd",BoatCmd,callback_manual)
    rospy.Subscriber("auto_cmd",BoatCmd,callback_auto)
    rospy.Subscriber("auto_state",String,callback_state)
    pub_cmd = rospy.Publisher('boatcmd', BoatCmd, queue_size = 10)
    rospy.spin()
    # cmd = BoatCmd()
    # listen_and_publish()