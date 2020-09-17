#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include <algorithm>


sensor_msgs::LaserScan scan_pro,last_scan;
int state = 0;
int fliter_n = 2;
int thres_number =10;
double thres_min = 0.2;
double thres_max = 1;
int thres = 15;
int fliter_mode = 1;

void fliter_scan(sensor_msgs::LaserScan& scan_msg)
{
    size_t i,j;
    int len = scan_msg.ranges.size();
    size_t n = fliter_n;
    float ranges[len+2*n], ranges_sort[2*n+1];
    
    for(i = 0; i<n; i++)
    {
        ranges[i] = scan_msg.ranges[len-n+i];
        ranges[len+n+i] = scan_msg.ranges[i];
    }
    for(i=0 ; i<len ; i++)
    {
        ranges[i+n]=scan_msg.ranges[i];
    }
    if(fliter_mode == 1)
    {
        for(i = 0; i < len; i++)
        {
            
            for(j = 0; j<(2*n+1); j++)
            {
                ranges_sort[j] = ranges[i+j];
            }
            std::sort(ranges_sort,ranges_sort+2*n+1);
            scan_msg.ranges[i] = ranges_sort[n];
        }
    }
    else if(fliter_mode == 2)
    {
        double sum = 0;
        for(i = 0; i < len; i++)
        {
            sum = 0;
            for(j = 0; j<(2*n+1); j++)
            {
                sum += ranges[i+j];
            }
            scan_msg.ranges[i] = sum/(2*n+1);
        }
    }
}

void cluster(sensor_msgs::LaserScan& scan_msg)
{
    int i = 0, j = 0, p_start, p_stop, step;
    int len = scan_msg.ranges.size();
    int cluster_n = 1;
    int thres_pn =thres_number;
    int thres_n = thres;
    double distance_q,thres;
    
    int point_n[1440];

    fliter_scan(scan_msg);
    for( i = 0; i < len; i++)//找第一个点
    {
        if(scan_msg.ranges[i] >= scan_msg.range_min && scan_msg.ranges[i] <= scan_msg.range_max ) break;
        scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
        scan_msg.intensities[i] = 0;
    }
    if(i < len)//判断是否没有点
    {
        scan_msg.intensities[i] = cluster_n;
        point_n[cluster_n] = 1;
        p_stop = len-1;
        p_start = i;
        if(p_start <= thres_n)
        {
            for( j = len-1; j > p_start; j--)
            {
                if(scan_msg.ranges[j] < scan_msg.range_min || scan_msg.ranges[j] > scan_msg.range_max)
                {
                    scan_msg.ranges[j] = std::numeric_limits<float>::infinity();
                    scan_msg.intensities[j] = 0;
                    continue;
                }
                step = (i>j) ? (i-j) : (i+len-j);
                if( step <= thres_n)
                {
                    thres = (scan_msg.ranges[i] + scan_msg.ranges[j]) / 2 * 0.00436 * thres_n;
                    //thres = (thres < thres_min) ? thres_min : (thres > thres_max ? thres_max : thres);
                    distance_q = scan_msg.ranges[i] * scan_msg.ranges[i] + scan_msg.ranges[j] * scan_msg.ranges[j]
                                 - 2 * scan_msg.ranges[i] * scan_msg.ranges[j] * std::cos(step*0.00436);
                    if(distance_q > (thres*thres)) break;
                    scan_msg.intensities[j] = cluster_n;
                    point_n[cluster_n]++;
                }
                else break;
                i = j;
            }
            p_stop = j;
            i = p_start;
        }
        for( j = p_start+1; j <= p_stop; j++)
        {
            if(scan_msg.ranges[j] < scan_msg.range_min || scan_msg.ranges[j] > scan_msg.range_max)
            {
                scan_msg.ranges[j] = std::numeric_limits<float>::infinity();
                scan_msg.intensities[j] = 0;
                continue;
            }
            if((j-i) <= thres_n)
            {
                thres = (scan_msg.ranges[i] + scan_msg.ranges[j]) / 2 * 0.00436 * thres_n;
                //thres = (thres < thres_min) ? thres_min : (thres > thres_max ? thres_max : thres);
                distance_q = scan_msg.ranges[i] * scan_msg.ranges[i] + scan_msg.ranges[j] * scan_msg.ranges[j]
                                - 2 * scan_msg.ranges[i] * scan_msg.ranges[j] * std::cos((j-i)*0.00436);
                if(distance_q > (thres*thres)) 
                {
                   cluster_n += 1;
                   point_n[cluster_n] = 1;
                }
            } 
            else 
            {
                cluster_n += 1;
                point_n[cluster_n] = 1;
            }
            scan_msg.intensities[j] = cluster_n;
            point_n[cluster_n]++;
            i = j;           
        }
    }
    last_scan.ranges.clear();
    for( i = 0; i < len; i++)
    {
        if(scan_msg.intensities[i] != 0.0 && point_n[(int)scan_msg.intensities[i]]<=thres_pn)
        {
            scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
            scan_msg.intensities[i] = 0.0;
        }
        last_scan.ranges.push_back(scan_msg.ranges[i]);
        last_scan.intensities.push_back(scan_msg.intensities[i]);
    }
   
    /*for( i = 0; i < len; i++)
    {
        if(scan_msg.intensities[i] == 0.0) continue;
        scan_msg.intensities[i] =((((int)scan_msg.intensities[i]) % 2)==1)?scan_msg.intensities[i]:(cluster_n - scan_msg.intensities[i]);
    }*/
}

/*void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    scan_pro.header.stamp = ros::Time::now();
    scan_pro.header.frame_id = scan->header.frame_id;
    scan_pro.angle_min = scan->angle_min;
    scan_pro.angle_max = scan->angle_max;
    scan_pro.angle_increment=scan->angle_increment;
    scan_pro.scan_time = scan->scan_time;
    scan_pro.time_increment = scan->time_increment;
    scan_pro.range_min = scan->range_min;
    scan_pro.range_max = scan->range_max;
    scan_pro.ranges = scan->ranges;
    scan_pro.intensities = scan->intensities;
    cluster(scan_pro);
    state = 1;
}*/

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    pub = n.advertise<sensor_msgs::LaserScan>("scan_pro", 1000);
    sub = n.subscribe("scan", 1000, &SubscribeAndPublish::callback, this);
  }

  void callback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    scan_pro.header.stamp = ros::Time::now();
    scan_pro.header.frame_id = scan->header.frame_id;
    scan_pro.angle_min = scan->angle_min;
    scan_pro.angle_max = scan->angle_max;
    scan_pro.angle_increment = scan->angle_increment;
    scan_pro.scan_time = scan->scan_time;
    scan_pro.time_increment = scan->time_increment;
    scan_pro.range_min = scan->range_min;
    scan_pro.range_max = scan->range_max;
    scan_pro.ranges = scan->ranges;
    scan_pro.intensities = scan->intensities;

    cluster(scan_pro);
    pub.publish(scan_pro);
  }

private:
  ros::NodeHandle n; 
  ros::Publisher pub;
  ros::Subscriber sub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_pro");
    ros::NodeHandle nh_private("~");
    nh_private.param<int>("fliter_n", fliter_n, 2);
    nh_private.param<int>("thres", thres, 15);
    nh_private.param<int>("thres_number", thres_number, 10);
    /*ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("scan", 1000, scanCallback);
    ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan_pro", 1000);

    ros::Rate loop_rate(15);
    while (n.ok()) 
    {    
        ros::spinOnce();
        if(state == 1)
        {
            pub.publish(scan_pro);
            state = 0;
        }
        loop_rate.sleep();
    }
    return 0;*/
    SubscribeAndPublish SAPObject;
    ros::spin();
    return 0;
}