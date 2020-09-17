#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "tf/transform_listener.h"
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <algorithm>
#include <string>

visualization_msgs::Marker  line_list;
geometry_msgs::Point p1,p2,p3,p4;
double roll, pitch, yaw;
std::string scan_id = "scan";
void compute_m(float* arr,int len, float* min ,float* max){
    if(len!=0)
    {
        *min = arr[0];
        *max = arr[0];
    }
    for(int i=1;i<len;i++)
    {
        if (*min>arr[i])
        {
            *min = arr[i];
        }

        if (*max<arr[i])
        {
            *max = arr[i];
        }
    }
}

void min_rect(float* x, float * y,size_t len)
{
    float x_temp[len],y_temp[len],s=0,min_s=2500.0;
    int index=0;
    float tt =0,max_x,max_y,min_x,min_y,dt=0.05;
    float p1_x,p1_y,p2_x,p2_y,p3_x,p3_y,p4_x,p4_y;
    for(size_t i=0;i<90;i++)
    {
        for(size_t j = 0;j<len;j++)
        {
            tt=i*M_PI/180.0;
            x_temp[j] = std::cos(tt)*x[j]-std::sin(tt)*y[j];
            y_temp[j] = std::sin(tt)*x[j]+std::cos(tt)*y[j];
        }
        compute_m(x_temp,len,&min_x,&max_x);
        compute_m(y_temp,len,&min_y,&max_y);
        s=(max_x-min_x)*(max_y-min_y);
        if(s<min_s)
        {
            min_s = s;
            index = i;
            p1_x = min_x-dt;
            p1_y = min_y-dt;
            p2_x = min_x-dt;
            p2_y = max_y+dt;
            p3_x = max_x+dt;
            p3_y = max_y+dt;
            p4_x = max_x+dt;
            p4_y = min_y-dt;
        }
    }
    tt = -index*M_PI/180.0;
    p1.x = std::cos(tt)*p1_x-std::sin(tt)*p1_y;
    p1.y = std::sin(tt)*p1_x+std::cos(tt)*p1_y;
    p1.z = 0;
    p2.x = std::cos(tt)*p2_x-std::sin(tt)*p2_y;
    p2.y = std::sin(tt)*p2_x+std::cos(tt)*p2_y;
    p2.z = 0;
    p3.x = std::cos(tt)*p3_x-std::sin(tt)*p3_y;
    p3.y = std::sin(tt)*p3_x+std::cos(tt)*p3_y;
    p3.z = 0;
    p4.x = std::cos(tt)*p4_x-std::sin(tt)*p4_y;
    p4.y = std::sin(tt)*p4_x+std::cos(tt)*p4_y;
    p4.z = 0;   
}


float dt = 0.03;
int state = 0;
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)//10ms
{
    
    line_list.points.clear();

    line_list.header.frame_id = "base_link";//scan->header.frame_id;
    line_list.header.stamp = scan->header.stamp;
    double angle_min = scan->angle_min + yaw;
    int count = 1440;
    float x[1440],temp_x[1440];
    float y[1440],temp_y[1440];
    size_t i=0,j=0;
    float number_np = 1.0;
    //ROS_INFO("1");
    for(i=0;i<count;i++)
    {
        if(scan->intensities[i] != 0.0)
        {
            x[i]=scan->ranges[i] * std::cos(angle_min + scan->angle_increment * i);
            y[i]=scan->ranges[i] * std::sin(angle_min + scan->angle_increment * i);
        }
    }
    for(i=0;i<count;i++)
    {
        if(scan->intensities[i] != 0.0 && scan->intensities[i] > number_np)
        {
           number_np = scan->intensities[i];
           temp_x[j]=x[i];
           temp_y[j]=y[i];
           j=1;
           break;
        }
    }
    i++;
    for(;i<count;i++)
    {
        if(scan->intensities[i] == number_np)
        {
           temp_x[j]=x[i];
           temp_y[j]=y[i];
           j++;
        }
        else if(scan->intensities[i] != 0.0 && scan->intensities[i] > number_np)
        {
            /*std::sort(temp_x,temp_x+j);
            std::sort(temp_y,temp_y+j);
            p1.x = temp_x[0]-dt;
            p1.y = temp_y[0]-dt;
            p1.z = 0;
            p2.x = temp_x[0]-dt;
            p2.y = temp_y[j-1]+dt;
            p2.z = 0;
            p3.x = temp_x[j-1]+dt;
            p3.y = temp_y[j-1]+dt;
            p3.z = 0;
            p4.x = temp_x[j-1]+dt;
            p4.y = temp_y[0]-dt;
            p4.z = 0;*/
            min_rect(temp_x,temp_y,j);
            line_list.points.push_back(p1);
            line_list.points.push_back(p2);
            line_list.points.push_back(p2);
            line_list.points.push_back(p3);
            line_list.points.push_back(p3);
            line_list.points.push_back(p4);
            line_list.points.push_back(p4);
            line_list.points.push_back(p1);
            number_np = scan->intensities[i];
            temp_x[0]=x[i];
            temp_y[0]=y[i];
            j=1;
            //ROS_INFO("[%f,%f,%f,%f,%f,%f,%f,%f]\n",p1.x,p1.y,p2.x,p2.y,p3.x,p3.y,p4.x,p4.y);     
        }
    }
    if(j!=0)
    {
        /*std::sort(temp_x,temp_x+j);
        std::sort(temp_y,temp_y+j);
        p1.x = temp_x[0]-dt;
        p1.y = temp_y[0]-dt;
        p1.z = 0;
        p2.x = temp_x[0]-dt;
        p2.y = temp_y[j-1]+dt;
        p2.z = 0;
        p3.x = temp_x[j-1]+dt;
        p3.y = temp_y[j-1]+dt;
        p3.z = 0;
        p4.x = temp_x[j-1]+dt;
        p4.y = temp_y[0]-dt;
        p4.z = 0;*/
        min_rect(temp_x,temp_y,j);
        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
        line_list.points.push_back(p2);
        line_list.points.push_back(p3);
        line_list.points.push_back(p3);
        line_list.points.push_back(p4);
        line_list.points.push_back(p4);
        line_list.points.push_back(p1);
        //ROS_INFO("[%f,%f,%f,%f,%f,%f,%f,%f]\n",p1.x,p1.y,p2.x,p2.y,p3.x,p3.y,p4.x,p4.y);  
    }

    j = 0;
    number_np =1.0;
    for(i=0;i<count;i++)
    {
        if(scan->intensities[i] == number_np )
        {
           temp_x[j]=x[i];
           temp_y[j]=y[i];
           j++;
           //ROS_INFO(": [%f,%f,%f]",(float)i,x[i],y[i]);
        }
    }
    if(j>0)
    {
        /*std::sort(temp_x,temp_x+j);
        std::sort(temp_y,temp_y+j);
        p1.x = temp_x[0]-dt;
        p1.y = temp_y[0]-dt;
        p1.z = 0;
        p2.x = temp_x[0]-dt;
        p2.y = temp_y[j-1]+dt;
        p2.z = 0;
        p3.x = temp_x[j-1]+dt;
        p3.y = temp_y[j-1]+dt;
        p3.z = 0;
        p4.x = temp_x[j-1]+dt;
        p4.y = temp_y[0]-dt;
        p4.z = 0;*/
        min_rect(temp_x,temp_y,j);
        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
        line_list.points.push_back(p2);
        line_list.points.push_back(p3);
        line_list.points.push_back(p3);
        line_list.points.push_back(p4);
        line_list.points.push_back(p4);
        line_list.points.push_back(p1);
        //ROS_INFO("[%f,%f,%f,%f,%f,%f,%f,%f]\n",p1.x,p1.y,p2.x,p2.y,p3.x,p3.y,p4.x,p4.y);  
    }
    state = 1;
    //ROS_INFO("2");
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "rectangular");
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("scan_id", scan_id, "scan");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>(scan_id, 100, scanCallback);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("lines", 10);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    while (n.ok())
    {
        
        try
        {
            listener.waitForTransform("/base_link", "/laser", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/base_link", "/laser", ros::Time(0), transform);
            break;
        }
        catch (tf::TransformException &ex) 
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }
    tf::Matrix3x3 m(transform.getRotation());
    m.getRPY(roll,pitch,yaw);
    ROS_INFO("base_link_to laser [roll: %f, pitch: %f, yaw: %f]",(float)roll,(float)pitch,(float)yaw/M_PI*180);

    line_list.ns = "lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    //宽0.1
    line_list.scale.x = 0.03;
    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    ros::Rate loop_rate(15);

    while (n.ok()) 
    {    
        ros::spinOnce();
        if(state == 1)
        {
            marker_pub.publish(line_list);
            state = 0;
        }
        
        loop_rate.sleep();
    }
}