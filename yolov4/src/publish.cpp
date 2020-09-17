#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<stdio.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle n; 
    ros::Time time = ros::Time::now();
    ros::Rate loop_rate(0.1);
    
     // 定义节点句柄   
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("/mynteye/left_rect/image_rect", 1);
    sensor_msgs::ImagePtr msg;

    cv::Mat frame;

    cv::Mat origin_image = cv::imread("/home/hzh/Desktop/catk/src/yolov4/imgs/0.jpeg");
    cv::cvtColor(origin_image, frame, cv::COLOR_BGR2RGB);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    
    while (ros::ok()){
    pub.publish(msg);
    loop_rate.sleep();
    ros::spinOnce();
    }
    return 0;
}