#include <curl/curl.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <jsoncpp/json/json.h>
#include "std_msgs/UInt16MultiArray.h"

unsigned long  getValueFromString(const char* rawArray, int begin, int end)
{
    char t_arr[64] = {0};
    int length = end - begin;
    memcpy(t_arr, rawArray + begin, length);
    return std::strtoul(t_arr, 0, 16);
}

void commchkCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg)
{

}

size_t write_data(char* buffer , size_t size, size_t nmemb, void* userp)
{
    try {
        Json::Reader reader;
        Json::Value root;
        bool res = reader.parse(buffer, buffer + (size * nmemb), root);
        if(!res)
            ROS_INFO("JSON Parsing Failed");
        std::string orderValue = root.get("order", "UTF-8").asString();
        int is4GComm = getValueFromString(orderValue.c_str(), 1, 2);
        char strData[64] = {0};
        if(is4GComm == 1)
            sprintf(strData, "%c%c%c%c%c%c", 165, 90, 4, 0, 4, 170);
        else
            sprintf(strData, "%c%c%c%c%c%c", 165, 90, 4, 1, 5, 170);
        std_msgs::String cmdData;
        cmdData.data.append(strData);
        ros::NodeHandle mcuNode;
        ros::Publisher mcuPub = mcuNode.advertise<std_msgs::String>("mcucommand", 10);
        mcuPub.publish(cmdData);
    } catch (std::exception& e) {

    }
    return size * nmemb;
}

int main(int argc, char **argv)
{
    curl_global_init(CURL_GLOBAL_ALL);

    ros::init(argc, argv, "commchk");
    ros::NodeHandle commchkNode;
    ros::Subscriber cruiseSpeedSub =  commchkNode.subscribe("cruise_speed", 10, commchkCallback);
    ros::Rate rate(1);

    std::string url_str = "http://47.100.92.173:10000/switchCheck";
    std::string form_str = "id=wrc";

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

    while(!ros::isShuttingDown())
    {
        //发送请求
        curl_easy_perform(pCurl);
        rate.sleep();
    }
    return 0;
}
