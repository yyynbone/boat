#include <curl/curl.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <jsoncpp/json/json.h>

static char strData[64] = {0};

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

size_t write_data(char* buffer , size_t size, size_t nmemb, void* usrp)
{
    try {
        Json::Reader reader;
        Json::Value root;
        bool res = reader.parse(buffer, buffer + (size * nmemb), root);
        if(!res)
            ROS_INFO("JSON Parsing Failed");
        //ROS_INFO("JSON:%s", buffer);
        std::string msgValue = root.get("msg", "0000000001").asString();
        int isAutoTrash = getValueFromString(msgValue.c_str(), 1, 2);
        int leftServo = getValueFromString(msgValue.c_str(), 2, 4);
        int rightServo = getValueFromString(msgValue.c_str(), 4, 6);
        int motor = getValueFromString(msgValue.c_str(), 6, 8);
        int light = getValueFromString(msgValue.c_str(), 8, 10);
        int sumchk = (9 + 3 + isAutoTrash + leftServo + rightServo + motor + light) % 256;
        //ROS_INFO("isAuto:%d, leftServo:%d, rightServo:%d, motor:%d, light:%d, sumchk:%d\n", isAutoTrash, leftServo, rightServo, motor, light, sumchk);
        // str(bytearray([165, 90, 7, 2, 49, 200 -int(speed),int(turn),sumchk,170]))#toCommand(order);
        memset(strData, 0, 64);
        //sprintf(strData, "%02x%02x%02x%02x%02x%02x%02x%02x%02x", 165, 90, 7,2,49, 200 - speed, turn, sumchk, 170);
        sprintf(strData, "%c%c%c%c%c%c%c%c%c%c%c", 165, 90, 9, 3, isAutoTrash, leftServo, rightServo, motor, light, sumchk, 170);

    }
    catch (std::exception& e) {
//        char stopcmd[64] = {0};
//        strcpy(stopcmd,toCommand(4));
//        std_msgs::String stopData;
//        stopData.data.append(stopcmd);
//        ros::NodeHandle mcuNode;
//        ros::Publisher mcuPub = mcuNode.advertise<std_msgs::String>("mcucommand", 10);
//        mcuPub.publish(stopData);
    }

    return size * nmemb;
}

int main(int argc, char **argv)
{
    curl_global_init(CURL_GLOBAL_ALL);

    ros::init(argc, argv, "trash");
    ros::NodeHandle mcuNode;
    ros::Publisher mcuPub = mcuNode.advertise<std_msgs::String>("mcucommand", 10);

    ros::Rate rate(10);

    std::string url_str = "http://47.100.92.173:10000/trashCheck";
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
    //回调参数
    //curl_easy_setopt(pCurl, CURLOPT_WRITEDATA, &mcuNode);
    //设置为Post
    curl_easy_setopt(pCurl, CURLOPT_POST, 1);
    //设置表单
    curl_easy_setopt(pCurl, CURLOPT_POSTFIELDS, form_str.c_str());

    while(!ros::isShuttingDown())
    {
        //发送请求
        curl_easy_perform(pCurl);
        //res=curl_easy_getinfo(pCurl, CURLINFO_RESPONSE_CODE, &res_code);
        std_msgs::String cmdData;
        cmdData.data.insert(0,strData,11);      //11是消息长度
        mcuPub.publish(cmdData);
        rate.sleep();
    }
    return 0;
}
