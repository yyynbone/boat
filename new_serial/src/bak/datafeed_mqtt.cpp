#include <curl/curl.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16MultiArray.h>
#include <jsoncpp/json/json.h>

int state = 0;

void sensorCallback(const std_msgs::String::ConstPtr& msg)
{
    const char *msgRaw = msg->data.c_str();

    char checkCode[4]={0};
    char newCheckCode[4] = {0};
    memcpy(checkCode, msgRaw+48, 2);
    memcpy(checkCode, msgRaw+48, 2);

    unsigned long checkCodeNum = std::strtoul(checkCode, 0, 16);
    int x = (checkCodeNum + state) % 256;
    snprintf(newCheckCode, 3, "%x", x);

    char stateBase[4]={0};
    snprintf(stateBase, 3, "0%x", state);

    char msgCvt[64] = {0};
    memcpy(msgCvt, msgRaw, 48);
    memcpy(msgCvt+48, stateBase, 2);
    memcpy(msgCvt+50, newCheckCode, 2);

    ROS_INFO("msgCvt:%s length:%d", msgCvt, sizeof(msgCvt));
}

void stateCallback(const std_msgs::String::ConstPtr& msg)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "datafeed");
    ros::NodeHandle sensorNode;
    ros::NodeHandle stateNode;
    ros::Subscriber sensorSub = sensorNode.subscribe("sensor", 1000, sensorCallback);
    ros::Subscriber stateSub = stateNode.subscribe("state", 1000, stateCallback);
    ros::spin();
    return 0;
}
