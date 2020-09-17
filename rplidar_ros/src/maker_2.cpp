#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include <algorithm>

visualization_msgs::Marker  line_list;
float dt = 0.03;
int state = 0;
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    
    
    line_list.points.clear();

    line_list.header.frame_id = scan->header.frame_id;
    line_list.header.stamp = scan->header.stamp;

    int count = 1440;
    float x[1440],temp_x[1440];
    float y[1440],temp_y[1440];
    geometry_msgs::Point p1,p2,p3,p4;
    size_t i=0,j=0;
    float number_np = 1.0;

    for(i=0;i<count;i++)
    {
        if(scan->intensities[i] != 0.0)
        {
            x[i]=scan->ranges[i] * std::cos(scan->angle_min + scan->angle_increment * i);
            y[i]=scan->ranges[i] * std::sin(scan->angle_min + scan->angle_increment * i);
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
            std::sort(temp_x,temp_x+j);
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
            p4.z = 0;
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
        std::sort(temp_x,temp_x+j);
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
        p4.z = 0;
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
        std::sort(temp_x,temp_x+j);
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
        p4.z = 0;
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
}


int main( int argc, char** argv )
{
    ros::init(argc, argv, "rectangular");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 100, scanCallback);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("lines", 10);

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