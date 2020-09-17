#include <curl/curl.h>
#include <ros/ros.h>
#include <thread>
#include <std_msgs/String.h>
#include <mutex>
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

#define WEIGHT 1

int state = 0;
static int yaw_mag_correct = 0;
static int yaw_GPS_last = 0;
static int yaw_mag_last = 0;
static int yaw_last = 0;
char msgCvt[64] = {0};

std::mutex lock;

unsigned long  getValueFromString(char* rawArray, int begin, int end)
{
    char t_arr[64] = {0};
    int length = end - begin;
    memcpy(t_arr, rawArray + begin, length);
    return std::strtoul(t_arr, 0, 16);
}

void httpThread()
{
    CURL *pCurl = curl_easy_init();
    if(pCurl == NULL)
    {
        std::cout<<"Init Curl failed."<<std::endl;
    }

    std::string url_str = "http://47.100.92.173:10000/gps";
    lock.lock();
    std::string form_str = "id=wrc&msg=" + std::string(msgCvt);
    lock.unlock();
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

    if(res != CURLE_OK)
    {
        // 获取详细错误信息
        const char* szErr = curl_easy_strerror(res);
        ROS_INFO("curl_easy_perform() failed: %s\n", szErr);
    }
    curl_easy_cleanup(pCurl);
}

void publishThread()
{
    ros::Rate rate(5);
    while(!ros::isShuttingDown())
    {
        //发布GPS主题
        ros::NodeHandle GPS_Node;
        ros::Publisher GPSPub = GPS_Node.advertise<std_msgs::Float64MultiArray>("GPS", 1000);

        std_msgs::Float64MultiArray GPS;
        lock.lock();
        double GPSN = getValueFromString(msgCvt,4, 12)/1000000.0;
        double GPSE = getValueFromString(msgCvt,12, 20)/1000000.0;
        lock.unlock();
        //ROS_INFO("GPSN:%f, GPSE:%f", GPSN, GPSE);
        GPS.data.push_back(GPSN);
        GPS.data.push_back(GPSE);
        GPSPub.publish(GPS);

        //发布YAW主题
        ros::NodeHandle yaw_Node;
        ros::Publisher yawPub = yaw_Node.advertise<std_msgs::Int16>("yaw", 10);

        //发布yaw_GPS
        ros::NodeHandle yawGPS_Node;
        ros::Publisher yawGPSPub = yawGPS_Node.advertise<std_msgs::Int16>("yaw_GPS", 10);

        //发布yaw_mag
        ros::NodeHandle yawMag_Node;
        ros::Publisher yawMagPub = yawMag_Node.advertise<std_msgs::Int16>("yaw_mag", 10);

        //发布电压主题
        ros::NodeHandle voltage_Node;
        ros::Publisher voltage_Pub = voltage_Node.advertise<std_msgs::Int16>("voltage", 10);

        //发布前雷达
        ros::NodeHandle radarFront_Node;
        ros::Publisher radarFrontPub = radarFront_Node.advertise<std_msgs::Int16>("radar_front", 10);

        //发布后雷达
        ros::NodeHandle radarBack_Node;
        ros::Publisher radarBackPub = radarBack_Node.advertise<std_msgs::Int16>("radar_back", 10);

        //发布传感器
        ros::NodeHandle sensorCheck_Node;
        ros::Publisher sensorCheckPub = sensorCheck_Node.advertise<std_msgs::Int16>("sensor_check", 10);

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

        std_msgs::Int16 yawMsg;
        yawMsg.data = yaw;
        yawPub.publish(yawMsg);
        //ROS_INFO("yawMsg:%d", yaw);

        std_msgs::Int16 yawGPSMsg;
        yawGPSMsg.data = yaw_GPS;
        yawGPSPub.publish(yawGPSMsg);
        //ROS_INFO("yawGPSMsg:%d", yaw_GPS);

        std_msgs::Int16 yawMagMsg;
        yawMagMsg.data = yaw_mag;
        yawMagPub.publish(yawMagMsg);
        //ROS_INFO("yawMagMsg:%d", yaw_mag);

        std_msgs::Int16 voltageMsg;
        voltageMsg.data = voltage;
        voltage_Pub.publish(voltageMsg);
        //ROS_INFO("voltageMsg:%d", voltage);

        std_msgs::Int16 radarFrontMsg;
        radarFrontMsg.data = radar_front;
        radarFrontPub.publish(radarFrontMsg);
        //ROS_INFO("radarFrontMsg:%d", radar_front);

        std_msgs::Int16 radarBackMsg;
        radarBackMsg.data = radar_back;
        radarBackPub.publish(radarBackMsg);
        //ROS_INFO("radarBackMsg:%d", radar_back);

        std_msgs::Int16 sensorCheckMsg;
        sensorCheckMsg.data = sensor_check;
        sensorCheckPub.publish(sensorCheckMsg);
        //ROS_INFO("sensorCheckMsg:%d", sensor_check);
        ros::spinOnce();
        rate.sleep();
    }
}

void sensorCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg,
                    ros::Publisher *GPSPub, ros::Publisher *yawPub, ros::Publisher *yawGPSPub, ros::Publisher *yawMagPub, ros::Publisher *voltage_Pub,
                    ros::Publisher *radarFrontPub, ros::Publisher *radarBackPub, ros::Publisher *sensorCheckPub)
{
  //const char *msgRaw = msg->data.c_str();
  //const unsigned char *msgRaw = msg->data.data();
  printf("--point 0 --:\n");
  char msgRaw[64] ={0};
  //std::string msgRaw;
  printf("\n--begin:\n");
  snprintf(msgRaw,48,"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
          msg->data.at(0), msg->data.at(1), msg->data.at(2), msg->data.at(3), msg->data.at(4), msg->data.at(5), msg->data.at(6), msg->data.at(7)
          , msg->data.at(8), msg->data.at(9), msg->data.at(10), msg->data.at(11), msg->data.at(12), msg->data.at(13), msg->data.at(14), msg->data.at(15)
          , msg->data.at(16), msg->data.at(17), msg->data.at(18), msg->data.at(19), msg->data.at(20), msg->data.at(21), msg->data.at(22), msg->data.at(23)
          , msg->data.at(24));

  printf("\n--end\n");
  char newCheckCode[4] = {0};
  printf("--point 1--\n");
  int x = (msg->data.at(24) + state) % 256;
  snprintf(newCheckCode, 3, "%02x", x);

  char stateBase[4]={0};
  snprintf(stateBase, 3, "02%x", state);

  lock.lock();
  memset(msgCvt, 0, 64);
  memcpy(msgCvt, msgRaw, 48);
  memcpy(msgCvt+48, stateBase, 2);
  memcpy(msgCvt+50, newCheckCode, 2);
  lock.unlock();
  //ROS_INFO("msgCvt:%s length:%d", msgCvt, sizeof(msgCvt));

  std_msgs::Float64MultiArray GPS;
  //lock.lock();
  double GPSN = getValueFromString(msgCvt,4, 12)/1000000.0;
  double GPSE = getValueFromString(msgCvt,12, 20)/1000000.0;
  //lock.unlock();
  //ROS_INFO("GPSN:%f, GPSE:%f", GPSN, GPSE);
  GPS.data.push_back(GPSN);
  GPS.data.push_back(GPSE);
  GPSPub->publish(GPS);

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

  std_msgs::Int16 yawMsg;
  yawMsg.data = yaw;
  yawPub->publish(yawMsg);
  //ROS_INFO("yawMsg:%d", yaw);

  std_msgs::Int16 yawGPSMsg;
  yawGPSMsg.data = yaw_GPS;
  yawGPSPub->publish(yawGPSMsg);
  //ROS_INFO("yawGPSMsg:%d", yaw_GPS);

  std_msgs::Int16 yawMagMsg;
  yawMagMsg.data = yaw_mag;
  yawMagPub->publish(yawMagMsg);
  //ROS_INFO("yawMagMsg:%d", yaw_mag);

  std_msgs::Int16 voltageMsg;
  voltageMsg.data = voltage;
  voltage_Pub->publish(voltageMsg);
  //ROS_INFO("voltageMsg:%d", voltage);

  std_msgs::Int16 radarFrontMsg;
  radarFrontMsg.data = radar_front;
  radarFrontPub->publish(radarFrontMsg);
  //ROS_INFO("radarFrontMsg:%d", radar_front);

  std_msgs::Int16 radarBackMsg;
  radarBackMsg.data = radar_back;
  radarBackPub->publish(radarBackMsg);
  //ROS_INFO("radarBackMsg:%d", radar_back);

  std_msgs::Int16 sensorCheckMsg;
  sensorCheckMsg.data = sensor_check;
  sensorCheckPub->publish(sensorCheckMsg);
  //ROS_INFO("sensorCheckMsg:%d", sensor_check);
  printf("--point 4--\n");
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

  //发布GPS主题
  ros::NodeHandle GPS_Node;
  ros::Publisher GPSPub = GPS_Node.advertise<std_msgs::Float64MultiArray>("GPS", 20);

  //发布YAW主题
  ros::NodeHandle yaw_Node;
  ros::Publisher yawPub = yaw_Node.advertise<std_msgs::Int16>("yaw", 20);

  //发布yaw_GPS
  ros::NodeHandle yawGPS_Node;
  ros::Publisher yawGPSPub = yawGPS_Node.advertise<std_msgs::Int16>("yaw_GPS", 20);

  //发布yaw_mag
  ros::NodeHandle yawMag_Node;
  ros::Publisher yawMagPub = yawMag_Node.advertise<std_msgs::Int16>("yaw_mag", 20);

  //发布电压主题
  ros::NodeHandle voltage_Node;
  ros::Publisher voltage_Pub = voltage_Node.advertise<std_msgs::Int16>("voltage", 20);

  //发布前雷达
  ros::NodeHandle radarFront_Node;
  ros::Publisher radarFrontPub = radarFront_Node.advertise<std_msgs::Int16>("radar_front", 20);

  //发布后雷达
  ros::NodeHandle radarBack_Node;
  ros::Publisher radarBackPub = radarBack_Node.advertise<std_msgs::Int16>("radar_back", 20);

  //发布传感器校验
  ros::NodeHandle sensorCheck_Node;
  ros::Publisher sensorCheckPub = sensorCheck_Node.advertise<std_msgs::Int16>("sensor_check", 20);

  //订阅传感器数据
  ros::Subscriber sensorSub = sensorNode.subscribe<std_msgs::UInt8MultiArray>("sensor",  0, boost::bind(&sensorCallback, _1,
                                                                              &GPSPub, &yawPub, &yawGPSPub, &yawMagPub, &voltage_Pub, &radarFrontPub, &radarBackPub, &sensorCheckPub));
  //订阅状态
  ros::Subscriber stateSub = stateNode.subscribe("state", 1000, stateCallback);
  //std::thread t0(httpThread);
  //t0.detach();
  //std::thread t1(publishThread);
  //t1.detach();

    ros::Rate rate(10);
    CURL *pCurl = curl_easy_init();
    if(pCurl == NULL)
    {
        std::cout<<"Init Curl failed."<<std::endl;
    }

    std::string url_str = "http://47.100.92.173:10000/gps";
    //ROS_INFO("form:%s\n", form_str.c_str());

    //请求超时时长
    curl_easy_setopt(pCurl, CURLOPT_TIMEOUT, 5L);
    //设置URL
    curl_easy_setopt(pCurl, CURLOPT_URL, url_str.c_str() );
    //设置为Post
    curl_easy_setopt(pCurl, CURLOPT_POST, 1);
    while(!ros::isShuttingDown())
    {
      std::string form_str = "id=wrc&msg=" + std::string(msgCvt);
      //设置表单
      curl_easy_setopt(pCurl, CURLOPT_POSTFIELDS, form_str.c_str());

      //发送请求
      CURLcode res = curl_easy_perform(pCurl);
      if(res != CURLE_OK)
      {
        // 获取详细错误信息
        const char* szErr = curl_easy_strerror(res);
      }
      rate.sleep();
      ros::spinOnce();
    }
  return 0;
}
