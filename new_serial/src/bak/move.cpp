#include <curl/curl.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16MultiArray.h>
#include <jsoncpp/json/json.h>

const char *toCommand(int i)
{
    std::string arr[5]={
        "\xA5\x5A\x05\x02\x38\x3F\xAA",
        "\xA5\x5A\x05\x02\x32\x39\xAA",
        "\xA5\x5A\x05\x02\x37\x3E\xAA",
        "\xA5\x5A\x05\x02\x39\x40\xAA",
        "\xA5\x5A\x05\x02\x35\x3C\xAA",
    };
    return arr[i].c_str();
}

unsigned long  getValueFromString(const char* rawArray, int begin, int end)
{
    char t_arr[64] = {0};
    int length = end - begin;
    memcpy(t_arr, rawArray + begin, length);
    return std::strtoul(t_arr, 0, 16);
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
        int isAutoPilot = getValueFromString(orderValue.c_str(), 1, 2);
        int speed = getValueFromString(orderValue.c_str(), 2, 4);
        int turn = getValueFromString(orderValue.c_str(), 4, 6);
        bool isUserOverride = false;
        if(isAutoPilot == 1)
        {
            if(speed != 100 && turn != 100)
                isUserOverride = true;
            else
                isUserOverride = false;
        }

        int sumchk = (7 + 2 + 49 + 200 - speed + turn) % 256;
        // str(bytearray([165, 90, 7, 2, 49, 200 -int(speed),int(turn),sumchk,170]))#toCommand(order);
        char strData[64] = {0};
        //sprintf(strData, "%02x%02x%02x%02x%02x%02x%02x%02x%02x", 165, 90, 7,2,49, 200 - speed, turn, sumchk, 170);
        sprintf(strData, "%c%c%c%c%c%c%c%c%c", 165, 90, 7, 2, 49, 200 - speed, turn, sumchk, 170);
        std_msgs::String cmdData;
        cmdData.data.append(strData);
        ros::NodeHandle mcuNode;
        ros::Publisher mcuPub = mcuNode.advertise<std_msgs::String>("mcucommand", 10);
        mcuPub.publish(cmdData);

        std_msgs::String stateData;
        stateData.data.push_back(isAutoPilot);
        ros::NodeHandle stateNode;
        ros::Publisher statePub = stateNode.advertise<std_msgs::String>("state", 1);
        statePub.publish(stateData);
    }
    catch (std::exception& e) {
        char stopcmd[64] = {0};
        strcpy(stopcmd,toCommand(4));
        std_msgs::String stopData;
        stopData.data.append(stopcmd);
        ros::NodeHandle mcuNode;
        ros::Publisher mcuPub = mcuNode.advertise<std_msgs::String>("mcucommand", 10);
        mcuPub.publish(stopData);
    }


    return size * nmemb;
}

void cruiseSpeedCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg)
{
    //int sumchk = (7 + 2 + 49 + 200 - msg.data.at + data.data[1]) % 256;
}

int main(int argc, char **argv)
{
    curl_global_init(CURL_GLOBAL_ALL);

    ros::init(argc, argv, "move");
    ros::NodeHandle cruiseSpeedNode;
    ros::Subscriber cruiseSpeedSub =  cruiseSpeedNode.subscribe("cruise_speed", 10, cruiseSpeedCallback);
    ros::Rate rate(5);

    std::string url_str = "http://47.100.92.173:10000/check";
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
    //curl_easy_setopt(pCurl, CURLOPT_WRITEDATA, )
    //设置为Post
    curl_easy_setopt(pCurl, CURLOPT_POST, 1);
    //设置表单
    curl_easy_setopt(pCurl, CURLOPT_POSTFIELDS, form_str.c_str());


    while(!ros::isShuttingDown())
    {
        //发送请求
        curl_easy_perform(pCurl);
        //res=curl_easy_getinfo(pCurl, CURLINFO_RESPONSE_CODE, &res_code);
        rate.sleep();
    }
    return 0;
}
