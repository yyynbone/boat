//
// Created by ling on 5/1/20.
//
#include <torch/torch.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sys/select.h>

#define kIMAGE_H 208
#define kIMAGE_W 384
#define kCHANNELS 3
#define CONF_THRES 0.5
#define COLOR 107, 142, 35  //bgr green

using namespace std;
//using namespace cv;
image_transport::Publisher pub;
torch::jit::script::Module module;






static void sleep_ms(unsigned int secs)

{

    struct timeval tval;

    tval.tv_sec=secs/1000;

    tval.tv_usec=(secs*1000)%1000000;

    select(0,NULL,NULL,NULL,&tval);

}


torch::Tensor preprocess(cv::Mat &frame)
{
    if (frame.empty() || !frame.data)
    {
        cout << "Frame empty!" << endl;
        exit(-1);
    }

    // resize
    cv::resize(frame, frame, cv::Size(kIMAGE_W, kIMAGE_H));

    // to Tensor
    torch::Tensor in_tensor = torch::from_blob(frame.data, {kIMAGE_H, kIMAGE_W, kCHANNELS}, torch::kByte);
    in_tensor = in_tensor.unsqueeze(0).permute({0, 3, 1, 2}).toType(torch::kFloat).div(255);

    return in_tensor;
}


cv::Mat postprocess(cv::Mat &frame, torch::Tensor &out_tensor)
{
    out_tensor = out_tensor.sigmoid().detach() > CONF_THRES;
    out_tensor = out_tensor.permute({0, 2, 3, 1}).squeeze().mul(255).clamp(0, 255).to(torch::kU8);
    out_tensor = out_tensor.to(torch::kCPU);
    // tensor to mat
    cv::Mat mask(kIMAGE_H, kIMAGE_W, CV_8UC1, out_tensor.data_ptr());
    return mask;
//    std::memcpy((void *) mask.data, out_tensor.data_ptr(), sizeof(torch::kU8) * out_tensor.numel());
//    cv::imshow("mask", mask);
//    cv::waitKey(0);
//    vector<vector<cv::Point>> contours;
//    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//
//    //find contours with max area
//    unsigned long max_contour = 0, max_index=0;
//    for(unsigned long i = 0; i < contours.size(); ++i){
//        if(max_contour < contours[i].size()){
//            max_contour = contours[i].size();
//            max_index = i;
//        }
//    }

//    //find bottom and top point in the max area
//    cv::Point bottom = cv::Point(kIMAGE_W/2, kIMAGE_H);  //for simplicity
//    auto top = contours[max_index][0];
//    for(const auto& coord : contours[max_index]){
//        if(top.y > coord.y){
//            top.y = coord.y;
//            top.x = coord.x;
//        }
//    }

//    cv::Mat canvas(mask.rows, mask.cols, CV_8UC3, cv::Scalar(0, 0, 0));
//    cv::drawContours(canvas, contours, int(max_index), cv::Scalar(COLOR), -1);

//    cv::blur(canvas, canvas, cv::Size(5, 5));
//    cv::line(canvas, top, bottom, cv::Scalar(0, 0, 255), 3);
//    //cv::Mat canvas;
//    //cv::cvtColor(mask, canvas, cv::COLOR_GRAY2BGR);
//    cv::addWeighted(frame, 0.5, canvas, 0.5, 0, frame);
//    cv::imshow("canvas", canvas);
//    cv::waitKey(0);

//    vector<cv::Point> endpoints{bottom, top};
//    return endpoints;
}


void predict(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        sensor_msgs::ImagePtr msgs1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
     pub.publish(msgs1);     
//    cv::Mat frame = cv::imread("/home/ling/Documents/CLionProjects/UNet-CPP/272.jpeg", cv::ImreadModes ::IMREAD_COLOR);

    //preprocess
    torch::Tensor in_tensor = preprocess(frame);
    //predict
    torch::Tensor out_tensor = module.forward({in_tensor.to(at::kCUDA)}).toTensor();
    //std::cout << out_tensor << std::endl;
    //postprocess
    cv::Mat mask = postprocess(frame, out_tensor);
        ROS_INFO("start");
    cv::imwrite("/home/hsm/3.jpeg",mask);
    ROS_INFO("saved");
    ROS_INFO("1");
    sleep_ms(200);
    sensor_msgs::ImagePtr msgs = cv_bridge::CvImage(std_msgs::Header(), "mono8", mask).toImageMsg();
    ROS_INFO("1");
    pub.publish(msgs);   
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "mask_predict");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    //image_transport::Publisher 
    
    
//    std::cout << "== Loading model...\n";
    //torch::jit::script::Module 
    module = torch::jit::load("/home/hsm/catkin_ws/src/water_surface/pth/E57_A0.96369.pt");  //*
//    std::cout << "== Model loaded!\n";
    const auto DEVICE = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;
    module.to(DEVICE);

    ROS_INFO("2");
    pub = it.advertise("image/mask", 1);
    image_transport::Subscriber sub = it.subscribe("/mynteye/left_rect/image_rect", 1, predict); //*
//    image_transport::Publisher pub = it.advertise("image/mask", 1);

    ros::spin();
}