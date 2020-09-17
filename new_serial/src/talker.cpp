#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::NodeHandle n2;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("sensor", 1000);
  ros::Publisher chatter_pub2 = n.advertise<std_msgs::String>("state", 1000);
  ros::Rate loop_rate(1000);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    std_msgs::String msg2;
    std::stringstream ss;
    std::stringstream ss2;
    ss << "ROS_MSG_1";
    ss2 << "ROS_MSG_2";
    msg.data = ss.str();
    msg2.data = ss2.str();
    //ROS_INFO("%s", msg.data.c_str());
    //ROS_INFO("%s", msg2.data.c_str());
    chatter_pub.publish(msg);
    chatter_pub2.publish(msg2);
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
