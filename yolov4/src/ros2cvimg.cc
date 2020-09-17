#include <iostream>
#include <ros/ros.h>
#include <chrono>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <torch/torch.h>
#include <opencv2/opencv.hpp>
#include "darknet.h"
#include "coco_names.h"

using namespace std;
int input_image_size = 416;
void image_callback(const sensor_msgs::ImageConstPtr& msg){
    
    float *point_arr;
    cv::Mat resized_image,img_float;
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8") -> image;
    ROS_INFO("sensor_msgs 2 cv mat");

    cv::resize(frame, resized_image, cv::Size(input_image_size, input_image_size));

    resized_image.convertTo(img_float, CV_32F, 1.0 / 255);

    auto img_tensor = torch::from_blob(img_float.data, {1, input_image_size, input_image_size, 3}).to(device);
    img_tensor = img_tensor.permute({0, 3, 1, 2});
    auto start = std::chrono::high_resolution_clock::now();
    auto result = net.predict(img_tensor, 80, 0.6, 0.4);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start); 
    std::cout<<  "time cost: " << duration.count()<< " ms\n";
    if (result.dim() == 1)
    {
        std::cout << "no object found" << std::endl;
    }
    else
    {
        int obj_num = result.size(0);

        std::cout << obj_num << " objects found" << std::endl;

        float w_scale = float(origin_image.cols) / input_image_size;
        float h_scale = float(origin_image.rows) / input_image_size;

        result.select(1, 1).mul_(w_scale);
        result.select(1, 2).mul_(h_scale);
        result.select(1, 3).mul_(w_scale);
        result.select(1, 4).mul_(h_scale);

        auto result_data = result.accessor<float, 2>();
    
        for (int i = 0; i < result.size(0); i++)
        {
        if (8==result_data[i][7])
        {
        cv::rectangle(origin_image, cv::Point(result_data[i][1], result_data[i][2]), cv::Point(result_data[i][3], result_data[i][4]), cv::Scalar(0, 0, 255), 1, 1, 0);
        
        
        for (int j =0 ; j<result_data[i].size(0);j++){
            std::cout << result_data[i][j] << std::endl;
            switch (j) {
                case 1:
                case 2:
                case 3:
                case 4:
                    *point_arr = result_data[i][j];
                    point_arr++;
                    break;
                default:
                    break;
            }
            
        }

        int clas_id = static_cast<size_t>(result_data[i][7]);
        float score = result_data[i][6];
        std::string text = coco_class_names[clas_id] + "-" + std::to_string(score);
        cv::putText(origin_image,
                    text,
                    cv::Point(result_data[i][1] + 5, result_data[i][2] + 5),   // Coordinates
                    cv::FONT_HERSHEY_COMPLEX_SMALL,  // Font
                    1.0,                             // Scale. 2.0 = 2x bigger
                    cv::Scalar(255, 100, 255));      // BGR Color
        
        cv::imwrite("../predict.jpg", origin_image);

        pub.publish(point_arr) ;

        }
        else
        {
            std::cout <<"none boat found"<< std::endl;
        }
        
    }
    }
    std::cout << "Done" << std::endl;
}