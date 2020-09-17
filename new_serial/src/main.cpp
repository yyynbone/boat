#include <curl/curl.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

#define WEIGHT 1

int state = 0;
static int yaw_mag_correct = 0;
static int yaw_GPS_last = 0;
static int yaw_mag_last = 0;
static int yaw_last = 0;

unsigned long  getValueFromString(char* rawArray, int begin, int end)
{
    char t_arr[64] = {0};
    int length = end - begin;
    memcpy(t_arr, rawArray + begin, length);
    return std::strtoul(t_arr, 0, 16);
}

void sensorCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{

  CURL *pCurl = curl_easy_init();
  if(pCurl == NULL)
  {
      std::cout<<"Init Curl failed."<<std::endl;
  }

  const unsigned char *msgRaw = msg->data.data();

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

  //ROS_INFO("msgCvt:%s length:%d", msgCvt, sizeof(msgCvt));

  std::string url_str = "http://47.100.92.173:10000/gps";
  std::string form_str = "id=wrc&msg=" + std::string(msgCvt);
  //ROS_INFO("form:%s\n", form_str.c_str());

  //请求超时时长
  curl_easy_setopt(pCurl, CURLOPT_TIMEOUT, 5L);
  //设置URL
  curl_easy_setopt(pCurl, CURLOPT_URL, url_str.c_str() );
  //设置为Post
  curl_easy_setopt(pCurl, CURLOPT_POST, 1);
  //设置表单
  curl_easy_setopt(pCurl, CURLOPT_POSTFIELDS, form_str.c_str());
  //发送请求
  CURLcode res = curl_easy_perform(pCurl);

//  res=curl_easy_getinfo(pCurl, CURLINFO_RESPONSE_CODE, &res_code);
  if(res != CURLE_OK)
  {
      // 获取详细错误信息
      const char* szErr = curl_easy_strerror(res);
      ROS_INFO("curl_easy_perform() failed: %s\n", szErr);
  }
  curl_easy_cleanup(pCurl);


  //发布GPS主题
  ros::NodeHandle GPS_Node;
  //std::msgs::Float64MultiArray GPS_DATA;
  ros::Publisher GPSPub = GPS_Node.advertise<std_msgs::Float64MultiArray>("GPS", 20);

  std_msgs::Float64MultiArray GPS;
  double GPSN = getValueFromString(msgCvt,4, 12)/1000000.0;
  double GPSE = getValueFromString(msgCvt,12, 20)/1000000.0;
  GPS.data.push_back(GPSN);
  GPS.data.push_back(GPSE);
  GPSPub.publish(GPS);

  //发布YAW主题
  ros::NodeHandle yaw_Node;
  ros::Publisher yawPub = yaw_Node.advertise<std_msgs::UInt8MultiArray>("yaw", 10);     //原python脚本中的类型是Int8,并非UInt8MultiArray

  //发布yaw_GPS
  ros::NodeHandle yawGPS_Node;
  ros::Publisher yawGPSPub = yawGPS_Node.advertise<std_msgs::UInt8MultiArray>("yaw_GPS", 10);

  //发布yaw_mag
  ros::NodeHandle yawMag_Node;
  ros::Publisher yawMagPub = yawMag_Node.advertise<std_msgs::UInt8MultiArray>("yaw_mag", 10);

  //发布电压主题
  ros::NodeHandle voltage_Node;
  ros::Publisher voltage_Pub = voltage_Node.advertise<std_msgs::UInt8MultiArray>("voltage", 10);

  //发布前雷达
  ros::NodeHandle radarFront_Node;
  ros::Publisher radarFrontPub = radarFront_Node.advertise<std_msgs::UInt8MultiArray>("radar_front", 10);

  //发布后雷达
  ros::NodeHandle radarBack_Node;
  ros::Publisher radarBackPub = radarBack_Node.advertise<std_msgs::UInt8MultiArray>("radar_back", 10);

  //发布传感器
  ros::NodeHandle sensorCheck_Node;
  ros::Publisher sensorCheckPub = sensorCheck_Node.advertise<std_msgs::UInt8MultiArray>("sensor_check", 10);

  int yaw = 0;
  int yaw_GPS = (getValueFromString(msgCvt, 24, 28) + 100) % 360;
  int yaw_mag = (getValueFromString(msgCvt, 28, 32)) % 360;

  int voltage = getValueFromString(msgCvt, 32, 36);
  int radar_front = getValueFromString(msgCvt, 36, 40);
  int radar_back = getValueFromString(msgCvt, 40, 44);
  int sensor_check = getValueFromString(msgCvt, 44, 46);

  int GPS_QF = getValueFromString(msgCvt, 46, 47);
  int GPS_Sat = getValueFromString(msgCvt, 47, 48);

  if(GPS_QF >= 4 && GPS_Sat >= 12)
  {
      yaw_mag_correct = yaw_GPS - yaw_mag;
      yaw = yaw_GPS;
  }
  else
  {
      yaw_mag = yaw_mag + yaw_mag_correct;
      int delta_yaw =  WEIGHT * ((yaw_mag - yaw_mag_last) % 360) + (1 - WEIGHT) * (yaw_GPS - yaw_GPS_last);
      yaw = (yaw_last + delta_yaw) % 360;
  }
  yaw_last = yaw;
  yaw_mag_last = yaw_mag;
  yaw_GPS_last = yaw_GPS;

  std_msgs::UInt8MultiArray yawMsg;
  yawMsg.data.push_back(yaw);
  yawPub.publish(yawMsg);

  std_msgs::UInt8MultiArray yawGPSMsg;
  yawGPSMsg.data.push_back(yaw_GPS);
  yawGPSPub.publish(yawGPSMsg);

  std_msgs::UInt8MultiArray yawMagMsg;
  yawMagMsg.data.push_back(yaw_mag);
  yawMagPub.publish(yawMagMsg);

  std_msgs::UInt8MultiArray voltageMsg;
  voltageMsg.data.push_back(voltage);
  voltage_Pub.publish(voltageMsg);

  std_msgs::UInt8MultiArray radarFrontMsg;
  radarFrontMsg.data.push_back(radar_front);
  radarFrontPub.publish(radarFrontMsg);

  std_msgs::UInt8MultiArray radarBackMsg;
  radarBackMsg.data.push_back(radar_back);
  radarBackPub.publish(radarBackMsg);

  std_msgs::UInt8MultiArray sensorCheckMsg;
  sensorCheckMsg.data.push_back(sensor_check);
  sensorCheckPub.publish(sensorCheckMsg);
}

void stateCallback(const std_msgs::String::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  state = std::strtoul(msg->data.c_str(), 0, 16);
}

int main(int argc, char **argv)
{
  curl_global_init(CURL_GLOBAL_ALL);
  //ROS_INFO("curl version:%s", curl_version());
  ros::init(argc, argv, "datafeed");
  ros::NodeHandle sensorNode;
  ros::NodeHandle stateNode;
  ros::Subscriber sensorSub = sensorNode.subscribe("sensor", 1000, sensorCallback);
  ros::Subscriber stateSub = stateNode.subscribe("state", 1000, stateCallback);
  ros::spin();
  return 0;
}
