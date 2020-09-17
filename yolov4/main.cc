#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <chrono>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <torch/torch.h>
#include <sys/select.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include "darknet.h"
#include "coco_names.h"

torch::Device device(torch::kCUDA);
Darknet net("/home/hsm/catkin_ws/src/yolov4/models/yolov4.cfg", &device);
int input_image_size;
ros::Publisher pub,pub1;
using namespace std;
static void sleep_ms(unsigned int secs)

{

	    struct timeval tval;

	        tval.tv_sec=secs/1000;

		    tval.tv_usec=(secs*1000)%1000000;

		        select(0,NULL,NULL,NULL,&tval);

}

void image_callback(const sensor_msgs::ImageConstPtr& msg){
  
    std_msgs::Float64MultiArray point_arr,origin_arr;
    vector<double> img_data;
    cv::Mat resized_image,img_float;
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8") -> image;
    //ROS_INFO("sensor_msgs 2 cv mat");

     //publish the origin array
    for (int a=0;a<frame.rows;a++)
        for (int b=0;b<frame.cols;b++)
            for (int c=0;c<frame.channels();c++){
                img_data.push_back(frame.at<cv::Vec3b>(a,b)[c]);
            }
    origin_arr.data = img_data;
    pub1.publish(origin_arr);

    cv::resize(frame, resized_image, cv::Size(input_image_size, input_image_size));
    cv::imwrite("../origin.jpg", frame);

    resized_image.convertTo(img_float, CV_32F, 1.0 / 255);  
    auto img_tensor = torch::from_blob(img_float.data, {1, input_image_size, input_image_size, 3}).to(device);
    img_tensor = img_tensor.permute({0, 3, 1, 2});
    //auto start = std::chrono::high_resolution_clock::now();
    auto result = net.predict(img_tensor, 80, 0.6, 0.4);
    //auto end = std::chrono::high_resolution_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start); 
    //std::cout<<  "time cost: " << duration.count()<< " ms\n";
    if (result.dim() == 1)
    {
        //std::cout << "no object found" << std::endl;
    }
    else
    {
        int obj_num = result.size(0);

        //std::cout << obj_num << " objects found" << std::endl;

        float w_scale = float(frame.cols) / input_image_size;
        float h_scale = float(frame.rows) / input_image_size;

        result.select(1, 1).mul_(w_scale);
        result.select(1, 2).mul_(h_scale);
        result.select(1, 3).mul_(w_scale);
        result.select(1, 4).mul_(h_scale);

        auto result_data = result.accessor<float, 2>();
    
        for (int i = 0; i < result.size(0); i++)
        {
        if (8==result_data[i][7])
        {
        cv::rectangle(frame, cv::Point(result_data[i][1], result_data[i][2]), cv::Point(result_data[i][3], result_data[i][4]), cv::Scalar(0, 0, 255), 1, 1, 0);
        
        
        for (int j =0 ; j<result_data[i].size(0);j++){
            std::cout << result_data[i][j] << std::endl;
            switch (j) {
                case 1:
                case 2:
                case 3:
                case 4:
                    point_arr.data.push_back(result_data[i][j]);
                    break;
                default:
                    break;
            }
            
        }

        int clas_id = static_cast<size_t>(result_data[i][7]);
        float score = result_data[i][6];
        std::string text = coco_class_names[clas_id] + "-" + std::to_string(score);
        cv::putText(frame,
                    text,
                    cv::Point(result_data[i][1] + 5, result_data[i][2] + 5),   // Coordinates
                    cv::FONT_HERSHEY_COMPLEX_SMALL,  // Font
                    1.0,                             // Scale. 2.0 = 2x bigger
                    cv::Scalar(255, 100, 255));      // BGR Color
        
        cv::imwrite("../predict.jpg", frame);

        pub.publish(point_arr) ;

        }
        else
        {
            //std::cout <<"none boat found"<< std::endl;
        }
        
    }
    }
    std::cout << "Done" << std::endl;
    sleep_ms(500);
}
int main(int argc, char* argv[])
{
  // std::cout << "hello\n";

  // if (argc != 2)
  // {
  //   std::cerr << "usage: yolov4 <image path>\n";
  //   return -1;
  // }
  //int input_image_size = 416;
  
  // std::cout << *argv << std::endl; //print the first arg
  //std::string cfg_file = argv[1];
  // torch::DeviceType device_type;
  // if(torch::cuda::is_available()){
  //   device_type = torch::kCUDA;
  //   std::cout << "load cuda" << std::endl;
  // }
  // else{
  //   device_type = torch::kCPU;
  //   std::cout << "load cuda" << std::endl;
  // }
  // torch::Device device(device_type);
  // std::string cfg_file = "/home/hsm/catkin_ws/src/yolov4/models/yolov4.cfg";
  // Darknet net(cfg_file.c_str(), &device);
  input_image_size = net.get_input_size();
  std::cout << "loading weight ..." << std::endl;
  //net.load_darknet_weights(argv[2]);
  net.load_darknet_weights("/home/hsm/catkin_ws/src/yolov4/models/yolov4.weights");
  std::cout << "weight loaded ..." << std::endl;
  
  net.to(device);
  torch::NoGradGuard no_grad;
  net.eval();
  ros::init(argc, argv, "boat_predict",ros::init_options::AnonymousName);
	ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  pub = n.advertise<std_msgs::Float64MultiArray>("/boat_pred/data",1);
  pub1 = n.advertise<std_msgs::Float64MultiArray>("/origin_data",1);
  image_transport::Subscriber sub = it.subscribe("/mynteye/left_rect/image_rect",1,image_callback);
  ros::spin();
  return 0;
  //cv::Mat origin_image, resized_image;
  //origin_image = cv::imread(argv[1]);
  //cv::cvtColor(origin_image, resized_image, cv::COLOR_BGR2RGB);

  
  // cv::resize(resized_image, resized_image, cv::Size(input_image_size, input_image_size));

  // cv::Mat img_float;
  // resized_image.convertTo(img_float, CV_32F, 1.0 / 255);

  // auto img_tensor = torch::from_blob(img_float.data, {1, input_image_size, input_image_size, 3}).to(device);
  // img_tensor = img_tensor.permute({0, 3, 1, 2});
  // auto start = std::chrono::high_resolution_clock::now();
  // auto result = net.predict(img_tensor, 80, 0.6, 0.4);
  // auto end = std::chrono::high_resolution_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start); 
  // std::cout<<  "time cost: " << duration.count()<< " ms\n";

  /*
  add ros msg to cv mat
  */


  // if (result.dim() == 1)
  // {
  //   std::cout << "no object found" << std::endl;
  // }
  // else
  // {
  //   int obj_num = result.size(0);

  //   std::cout << obj_num << " objects found" << std::endl;

  //   float w_scale = float(origin_image.cols) / input_image_size;
  //   float h_scale = float(origin_image.rows) / input_image_size;

  //   result.select(1, 1).mul_(w_scale);
  //   result.select(1, 2).mul_(h_scale);
  //   result.select(1, 3).mul_(w_scale);
  //   result.select(1, 4).mul_(h_scale);

  //   auto result_data = result.accessor<float, 2>();
 
  //   for (int i = 0; i < result.size(0); i++)
  //   {
  //     if (8==result_data[i][7])
  //     {
  //     cv::rectangle(origin_image, cv::Point(result_data[i][1], result_data[i][2]), cv::Point(result_data[i][3], result_data[i][4]), cv::Scalar(0, 0, 255), 1, 1, 0);
      
      
  //   cv::imwrite("../predict.jpg", origin_image);
  //     }
  //     else
  //     {
  //       std::cout <<"none boat found"<< std::endl;
  //     }
      
  // }
  // }
  // std::cout << "Done" << std::endl;
}
