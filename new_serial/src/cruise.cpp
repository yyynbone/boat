#include <curl/curl.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <jsoncpp/json/json.h>
#include "std_msgs/Int16.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

#define PI 3.141592653

static bool flag_yaw;
static bool flag_gps;
static bool flag_set;

static int16_t yaw;
static float lon;
static float lat;
static int Cruise_Speed_L = 4500;
static int Cruise_Speed_R = 4500;
static int last_sum_lon;
static int last_sum_lat;
static int velocity;
static std::vector<float> realLon;
static std::vector<float> realLat;

unsigned long  getValueFromString(const char* rawArray, int begin, int end)
{
    char t_arr[64] = {0};
    int length = end - begin;
    memcpy(t_arr, rawArray + begin, length);
    return std::strtoul(t_arr, 0, 16);
}

int sum(std::list<int> &dataList)
{
    int sum = 0;
    for(std::list<int>::iterator it = dataList.begin(); it != dataList.end(); it++)
    {
        sum += sum + *it;
    }
    return sum;
}

size_t write_data(char* buffer , size_t size, size_t nmemb, void* userp)
{
    try {
        Json::Reader reader;
        Json::Value root;
        bool res = reader.parse(buffer, buffer + (size * nmemb), root);
        if(!res)
            ROS_INFO("JSON Parsing Failed");
        //printf("size:%d, nmemb:%d\n", size, nmemb);
        //ROS_INFO("CRUISE JSON:%s", buffer);
        std::string orderValue = root.get("msg", "00").asString();
        int msgLength = getValueFromString(orderValue.c_str(), 0, 2);
        if(orderValue.length() == msgLength * 16 + 2)
        {
           std::list<int> lonList;
           std::list<int> latList;
           latList.clear();
           lonList.clear();
           realLat.clear();
           realLon.clear();
           int sum_lon = 0;
           int sum_lat = 0;
           for(int i = 0; i < msgLength; i++)
           {
               int lat_t = getValueFromString(orderValue.c_str(), i * 16 + 2, i * 16 + 10);
               int lon_t = getValueFromString(orderValue.c_str(), i * 16 + 10, i * 16 + 18);
               sum_lat += lat_t;
               sum_lon += lon_t;

               latList.push_back(lat_t);
               lonList.push_back(lon_t);

               realLon.push_back(lon_t / 1000000.0);
               realLat.push_back(lat_t / 1000000.0);
           }
           if(sum_lon != last_sum_lon && sum_lat != last_sum_lat)
               velocity = 0;

           last_sum_lat = sum_lat;
           last_sum_lon = sum_lon;

           flag_set =  true;
        }
    } catch (std::exception& e) {

    }
    return size * nmemb;
}

float angleFromCoordinate(float lat_orig, float lon_orig, float lat_real, float lon_real)
{
    float d_lon = lon_real - lon_orig;
    float d_lat = lat_real - lat_orig;
    float lat_r = (lat_orig + lat_real) / 360 * PI;

    float y = cos(lat_r) * d_lon;
    float x = d_lat;

    float brng = (atan2(y, x) / PI ) * 180;
    if(brng < 0)
        brng = brng + 360;
    return brng;
}

int yawControl(float lat_orig, float lon_orig, float lat_real, float lon_real)
{
    float u_temp = 0.0;
    float err = angleFromCoordinate(lat_orig, lon_orig, lat_real, lon_real) - yaw;

    float err_sum = 0;
    //?????????
    while(err < -180)
        err = err + 360;
    while(err > 180)
        err = err - 360;

    if(err > -90 && err < 90)
        err_sum = err_sum + 0.1 * err;

    u_temp = 15 * err;

    if(err > -1 && err < 1)
        err_sum = 0;

    return u_temp;
}

void cruiseControl()
{
    if(velocity >= realLat.size())
        velocity = 0;
    if(realLat.size() > velocity || realLon.size() > velocity)
    {
        int u = yawControl(lat, lon, realLat[velocity], realLon[velocity]);
        if(abs(u) > 400)
        {
            //int fspeed = 0;
            Cruise_Speed_L = 4500 + 0 + u;
            Cruise_Speed_R = 4500 + 0 - u;
        }
        else
        {
            //int fspeed = 900;
            Cruise_Speed_L = 4500 + 900 + u;
            Cruise_Speed_R = 4500 + 900 - u;
        }

        if(Cruise_Speed_L>6000)
            Cruise_Speed_L=6000;
        else if(Cruise_Speed_L<3000)
            Cruise_Speed_L=3000;

        if(Cruise_Speed_R>6000)
            Cruise_Speed_R=6000;
        else if(Cruise_Speed_R<3000)
            Cruise_Speed_R=3000;

        if(abs(lat * 1000000 - realLat[velocity] * 1000000) <= 30 && abs(lon * 1000000 - realLon[velocity]) <= 40)
            velocity++;
    }
}

void get_set_msg()
{
    std::string url_str = "http://47.100.92.173:10000/allMsgCheck";
    std::string form_str = "id=wrc&&msg=msg";

    CURL *pCurl = curl_easy_init();
    if(pCurl == NULL)
    {
      std::cout<<"Init Curl failed."<<std::endl;
    }
    //请求超时时长
    curl_easy_setopt(pCurl, CURLOPT_TIMEOUT, 5L);
    //设置URL
    curl_easy_setopt(pCurl, CURLOPT_URL, url_str.c_str() );
    //设置回调
    curl_easy_setopt(pCurl, CURLOPT_WRITEFUNCTION, write_data);
    //设置为Post
    curl_easy_setopt(pCurl, CURLOPT_POST, 1);
    //设置表单
    curl_easy_setopt(pCurl, CURLOPT_POSTFIELDS, form_str.c_str());
    //发送请求
    curl_easy_perform(pCurl);
}

void yawCallback(const std_msgs::Int16::ConstPtr& msg)
{
    yaw = msg->data;
    flag_yaw = true;
}

void gpsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    lat = msg->data.at(0);
    lon = msg->data.at(1);
    flag_gps = true;
}

void stateCallback(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == "1")
    {
        get_set_msg();
    }
}

int main(int argc, char **argv)
{
  curl_global_init(CURL_GLOBAL_ALL);
  ros::init(argc, argv, "cruise");

  ros::NodeHandle   yawNode;
  ros::NodeHandle   GPSNode;
  ros::NodeHandle   stateNode;
  ros::NodeHandle   cruiseNode;
  ros::Publisher cruisePub = cruiseNode.advertise<std_msgs::UInt16MultiArray>("cruise_speed", 1);
  ros::Subscriber sensorSub = yawNode.subscribe("yaw", 1000, yawCallback);
  ros::Subscriber GPSSub = GPSNode.subscribe("GPS", 1000, gpsCallback);
  ros::Subscriber stateSub = stateNode.subscribe("state", 1000, stateCallback);

  ros::Rate rate(100);
  while(!ros::isShuttingDown())
  {
      if(flag_yaw && flag_gps)
      {
          std_msgs::UInt16MultiArray cruise_speed;
          flag_yaw = false;
          flag_gps = false;
          if(flag_set)
          {
              flag_set = false;
              //printf("****before*****");
              cruiseControl();
              //printf("****after*****");
          }
          cruise_speed.data.push_back((Cruise_Speed_L + Cruise_Speed_R - 6000) / 30);
          cruise_speed.data.push_back((Cruise_Speed_L - Cruise_Speed_R + 3000) / 30);
          cruisePub.publish(cruise_speed);
      }
      rate.sleep();
      ros::spinOnce();
  }
  return 0;
}
