#ifndef __ROS2CVIMG_H__
#define __ROS2CVIMG_H__

#include <iostream>
#include <ros/ros.h>
#include <chrono>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <torch/torch.h>
#include <opencv2/opencv.hpp>
#include "darknet.h"
#include "coco_names.h"
int input_image_size = 416;
void image_callback(const sensor_msgs::ImageConstPtr& msg);

#endif