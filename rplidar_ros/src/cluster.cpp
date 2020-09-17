#include "ros/ros.h"

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <boat_msgs/BoatCmd.h>
#include <boat_msgs/BoatVel.h>

#include "tf/transform_listener.h"
#include <tf/tf.h>
#include "cluster.h"

visualization_msgs::Marker line_list,next_list;

geometry_msgs::Point p1,p2,p3,p4;
sensor_msgs::LaserScan scan_pro,scan_now,scan_last;
double roll, pitch, yaw;

int fliter_n = 3, thres_s = 50, thres_l = 50, thres_number = 1;
double min_thres_s = 0.125, max_thres_s = 0.5,min_thres_l = 0.25, max_thres_l = 1.5;
bool display_s = false, display_l = true;

double p_x[1440], p_y[1440], p_x_last[1440], p_y_last[1440];
int p_z[1440],p_z_last[1440];

double speed_theta = 0.0, speed_x= 0.0, speed_y = 0.0, scan_time_last = 0.0, d_scan_time = 0.0;

float avoid_w = 0, avoid_x = 0;
int state_auto = 0;
float vel_h = 40, vel_l = 10;
float global_direction = 0;
int state_local_start = 0, state_local_stop = 1;

OBSTACLE_LIST obstacle_now_s,obstacle_now_l,obstacle_last_s,obstacle_last_l,obstacle_temp;
OBSTACLE_CLUSTER_LIST obstacle_list_now,obstacle_list_last;

int mode_1 = 1;

enum local_state local_state_now = local_stop;

void fliter_scan(sensor_msgs::LaserScan& scan_msg, size_t n, int fliter_mode){
    size_t i,j;
    int len = scan_msg.ranges.size();
    float ranges[len+2*n], ranges_sort[2*n+1];
    
    for(i = 0; i<n; i++){
        ranges[i] = scan_msg.ranges[len-n+i];
        ranges[len+n+i] = scan_msg.ranges[i];
    }
    for(i = 0 ; i<len; i++){
        ranges[i+n]=scan_msg.ranges[i];
    }
    if(fliter_mode == 1){
        for(i = 0; i < len; i++){
            for(j = 0; j<(2*n+1); j++){
                ranges_sort[j] = ranges[i+j];
            }
            std::sort(ranges_sort,ranges_sort+2*n+1);
            scan_msg.ranges[i] = ranges_sort[n];
        }
    }/*
    else if(fliter_mode == 2){
        double sum = 0;
        for(i = 0; i < len; i++){
            sum = 0;
            for(j = 0; j<(2*n+1); j++){
                sum += ranges[i+j];
            }
            scan_msg.ranges[i] = sum/(2*n+1);
        }
    }*/
}

void cluster_scan(sensor_msgs::LaserScan& scan_msg, OBSTACLE_LIST& obstacle_, int thres_n, double thres_min, double thres_max){
    int len = scan_msg.ranges.size();
    int i = 0, j = 0, p_start, p_stop, step;
    double distance_q,thres;
    obstacle p_obstacle;
    obstacle_.clear();
    for( i = 0; i < len; i++){
        if(scan_msg.ranges[i] >= scan_msg.range_min && scan_msg.ranges[i] <= scan_msg.range_max) break;
        scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
        scan_msg.intensities[i] = 0;
    }
    if(i < len){   
        p_obstacle.scan.index.push_back(i);
        p_obstacle.scan.number = 1;
        p_stop = len-1;
        p_start = i;
        if(p_start <= thres_n){
            for( j = len-1; j > p_start; j--){
                if(scan_msg.ranges[j] < scan_msg.range_min || scan_msg.ranges[j] > scan_msg.range_max){
                    scan_msg.ranges[j] = std::numeric_limits<float>::infinity();
                    scan_msg.intensities[j] = 0;
                    continue;
                }
                step = (i>j) ? (i-j) : (i+len-j);
                if( step <= thres_n){
                    thres = (scan_msg.ranges[i] + scan_msg.ranges[j]) / 2 * 0.00436 * thres_n;
                    thres = (thres < thres_min) ? thres_min : (thres > thres_max ? thres_max : thres);
                    distance_q = scan_msg.ranges[i] * scan_msg.ranges[i] + scan_msg.ranges[j] * scan_msg.ranges[j]
                                 - 2 * scan_msg.ranges[i] * scan_msg.ranges[j] * std::cos(step*0.00436);
                    if(distance_q > (thres*thres)) break;
                    p_obstacle.scan.index.push_front(j);
                    p_obstacle.scan.number++;
                    i = j;
                }
                else break; 
            }
            p_stop = j;
            i = p_start;
        }
        for( j = p_start+1; j <= p_stop; j++){
            if(scan_msg.ranges[j] < scan_msg.range_min || scan_msg.ranges[j] > scan_msg.range_max){
                scan_msg.ranges[j] = std::numeric_limits<float>::infinity();
                scan_msg.intensities[j] = 0;
                continue;
            }
            if((j-i) <= thres_n){
                thres = (scan_msg.ranges[i] + scan_msg.ranges[j]) / 2 * 0.00436 * thres_n;
                thres = (thres < thres_min) ? thres_min : (thres > thres_max ? thres_max : thres);
                distance_q = scan_msg.ranges[i] * scan_msg.ranges[i] + scan_msg.ranges[j] * scan_msg.ranges[j]
                                - 2 * scan_msg.ranges[i] * scan_msg.ranges[j] * std::cos((j-i)*0.00436);
                if(distance_q > (thres*thres)) {
                    obstacle_.push_back(p_obstacle);
                    p_obstacle.scan.index.clear();
                    p_obstacle.scan.number = 0;
                }
            } 
            else {
                obstacle_.push_back(p_obstacle);
                p_obstacle.scan.index.clear();
                p_obstacle.scan.number = 0;
            }
            p_obstacle.scan.index.push_back(j);
            p_obstacle.scan.number++;
            i = j;           
        }
        obstacle_.push_back(p_obstacle);
    }    
}

void scan2points(sensor_msgs::LaserScan& scan_msg, double* x, double * y, int len){
    for(size_t i = 0; i < len; i++){
        if(scan_msg.ranges[i] > scan_msg.range_min && scan_msg.ranges[i] < scan_msg.range_max){
            x[i] = scan_msg.ranges[i] * std::cos(scan_msg.angle_min + scan_msg.angle_increment * i);
            y[i] = scan_msg.ranges[i] * std::sin(scan_msg.angle_min + scan_msg.angle_increment * i);
        }
    }
}

void compute_m(double* arr, int len, double* min, double* max){
    if(len!=0){
        *min = arr[0];
        *max = arr[0];
    }
    for(int i=1;i<len;i++){
        if (*min>arr[i]){
            *min = arr[i];
        }
        if (*max<arr[i]){
            *max = arr[i];
        }
    }
}

double min(double x, double y){
    return x<y?x:y;
}
 
double max(double x, double y){
    return x>y?x:y;
}

double distance_points(p_point p1, p_point p2){
    return std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}

double mutiply_cross(double x1, double y1, double x2, double y2){
    return (x1*y2-y1*x2);
}

double mutiply_cross_p(p_point p1, p_point p2,p_point q1, p_point q2){
    return (p2.x-p1.x)*(q2.y-q1.y)-(p2.y-p1.y)*(q2.x-q1.x);
}

void p_now2last(double* now_, double* last_, int len){
    for(size_t i = 0; i<len; i++)last_[i] = now_[i];
}

void rect_rot(p_rect& rect, float angle){
    double tt = angle*M_PI/180.0;
    double x, y;
    for(size_t i=0; i<4; i++){
        x = rect.fixed_point[i].x;
        y = rect.fixed_point[i].y;
        rect.fixed_point[i].x = std::cos(tt)*x-std::sin(tt)*y;
        rect.fixed_point[i].y = std::sin(tt)*x+std::cos(tt)*y;
    }
}

void min_rect(double* x, double * y, size_t len, OBSTACLE_LIST::iterator& plist, double dt, double k){
    double x_temp[len],y_temp[len],s=0,min_s=2500.0;
    int index=0;
    double tt =0,max_x,max_y,min_x,min_y;//,dt=0.01;
    //double p1_x,p1_y,p2_x,p2_y,p3_x,p3_y,p4_x,p4_y;
    double px[4],py[4];
    for(size_t i=0;i<90;i++){
        for(size_t j = 0;j<len;j++){
            tt=i*M_PI/180.0;
            x_temp[j] = std::cos(tt)*x[j]-std::sin(tt)*y[j];
            y_temp[j] = std::sin(tt)*x[j]+std::cos(tt)*y[j];
        }
        compute_m(x_temp,len,&min_x,&max_x);
        compute_m(y_temp,len,&min_y,&max_y);
        s=(max_x-min_x)*(max_y-min_y);
        if(s<min_s){
            min_s = s;
            index = i;
            px[0] = min_x-dt;
            py[0] = min_y-dt;
            px[1] = min_x-dt;
            py[1] = max_y+dt;
            px[2] = max_x+dt;
            py[2] = max_y+dt;
            px[3] = max_x+dt;
            py[3] = min_y-dt;
        }
    }
    double l1 = px[3] - px[0], l2 = py[1] - py[0];
    double ddt;
    if(l1 > l2){
        plist->rect.W = l2;
        plist->rect.L = l1;
        plist->rect.angle = -index;
        ddt = l1*(k-1);
        px[0] -= ddt;
        px[1] -= ddt;
        px[2] += ddt;
        px[3] += ddt;
    }
    else{
        plist->rect.W = l1;
        plist->rect.L = l2;
        plist->rect.angle = 90-index;
        ddt = l2*(k-1);
        py[0] -= ddt;
        py[1] += ddt;
        py[2] += ddt;
        py[3] -= ddt;
    }
    //ROS_INFO("p1:%f,%f",px[0],py[0]);
    //ROS_INFO("p2:%f,%f",px[1],py[1]);
    //ROS_INFO("p3:%f,%f",px[2],py[2]);
    //ROS_INFO("p4:%f,%f",px[3],py[3]);
    tt = -index*M_PI/180.0;
    double distance_q[4];
    int min_index = 0, min_index_ = 0;
    double sum_x = 0.0, sum_y = 0.0;
    for(size_t k = 0; k < 4; k++){
        plist->rect.fixed_point[k].x = std::cos(tt)*px[k]-std::sin(tt)*py[k];
        plist->rect.fixed_point[k].y = std::sin(tt)*px[k]+std::cos(tt)*py[k];
        sum_x += plist->rect.fixed_point[k].x;
        sum_y += plist->rect.fixed_point[k].y;
        distance_q[k] = plist->rect.fixed_point[k].x*plist->rect.fixed_point[k].x
                        + plist->rect.fixed_point[k].y*plist->rect.fixed_point[k].y;
        if(distance_q[k] < distance_q[min_index])min_index = k;
    } 
    plist->rect.center_point.x = sum_x/4;
    plist->rect.center_point.y = sum_y/4;
    plist->rect.match_point.x = plist->rect.fixed_point[min_index].x;
    plist->rect.match_point.y = plist->rect.fixed_point[min_index].y;
    plist->rect.index = min_index;
    if(l1 > l2){
        switch(min_index){
            case 0:min_index_= 3;break;
            case 1:min_index_= 2;break;
            case 2:min_index_= 1;break;
            case 3:min_index_= 0;break;
        }
    }
    else{
        switch(min_index){
            case 0:min_index_= 1;break;
            case 1:min_index_= 0;break;
            case 2:min_index_= 3;break;
            case 3:min_index_= 2;break;
        }
    }
    plist->rect.match_point_.x = plist->rect.fixed_point[min_index_].x;
    plist->rect.match_point_.y = plist->rect.fixed_point[min_index_].y;
}

void rect_ob(OBSTACLE_LIST::iterator& plist, double dt, double k){
    size_t len = plist->scan.number;
    double *x = new double[len];
    double *y = new double[len];
    size_t i=0;
    //plist->scan.start_index = plist->scan.index.front();
    //plist->scan.stop_index = plist->scan.index.back();
    for(INT_LIST::iterator p = plist->scan.index.begin(); p != plist->scan.index.end(); p++){
        x[i] = p_x[*p];//scan_msg.ranges[*p] * std::cos(scan_msg.angle_min + scan_msg.angle_increment * (*p));
        y[i] = p_y[*p];//scan_msg.ranges[*p] * std::sin(scan_msg.angle_min + scan_msg.angle_increment * (*p));
        i++;
        //ROS_INFO("%f,%f",x[i],y[i]);
    }
    min_rect(x,y,len,plist,dt,k);
    delete [] x;
    delete [] y;

}

void rect_scan(sensor_msgs::LaserScan& scan_msg, OBSTACLE_LIST& obstacle_, double* p_x, double * p_y, double dt, double k)
{
    //size_t len,i;
    for(OBSTACLE_LIST::iterator plist = obstacle_.begin(); plist != obstacle_.end(); plist++)
    {
        /*len = plist->scan.number;
        double *x = new double[len];
        double *y = new double[len];
        i=0;
        //plist->scan.start_index = plist->scan.index.front();
        //plist->scan.stop_index = plist->scan.index.back();
        for(INT_LIST::iterator p = plist->scan.index.begin(); p != plist->scan.index.end(); p++)
        {
            x[i] = p_x[*p];//scan_msg.ranges[*p] * std::cos(scan_msg.angle_min + scan_msg.angle_increment * (*p));
            y[i] = p_y[*p];//scan_msg.ranges[*p] * std::sin(scan_msg.angle_min + scan_msg.angle_increment * (*p));
            i++;
            //ROS_INFO("%f,%f",x[i],y[i]);
        }
        min_rect(x,y,len,plist,dt);
        delete [] x;
        delete [] y;*/
        rect_ob(plist,dt,k);
    }
}

void maker_lines(OBSTACLE_LIST& obstacle_, bool clear_){
    if(clear_)line_list.points.clear();
    line_list.header.frame_id = "base_link";//scan->header.frame_id;
    line_list.header.stamp = ros::Time::now();
    for(OBSTACLE_LIST::iterator plist = obstacle_.begin(); plist != obstacle_.end(); plist++){
        p1.x = plist->rect.fixed_point[0].x;
        p1.y = plist->rect.fixed_point[0].y;
        p1.z = 0;
        p2.x = plist->rect.fixed_point[1].x;
        p2.y = plist->rect.fixed_point[1].y;
        p2.z = 0;
        p3.x = plist->rect.fixed_point[2].x;
        p3.y = plist->rect.fixed_point[2].y;
        p3.z = 0;
        p4.x = plist->rect.fixed_point[3].x;
        p4.y = plist->rect.fixed_point[3].y;
        p4.z = 0;
        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
        line_list.points.push_back(p2);
        line_list.points.push_back(p3);
        line_list.points.push_back(p3);
        line_list.points.push_back(p4);
        line_list.points.push_back(p4);
        line_list.points.push_back(p1);
        //p1.x = 0;
        //p1.y = 0;
        //p2.x = plist->rect.center_point.x;
        //p2.y = plist->rect.center_point.y;
        //p2.x = plist->rect.match_point.x;
        //p2.y = plist->rect.match_point.y;
        //line_list.points.push_back(p1);
        //line_list.points.push_back(p2);
        // p2.x = plist->rect.match_point_.x;
        // p2.y = plist->rect.match_point_.y;
        // line_list.points.push_back(p1);
        // line_list.points.push_back(p2);        
    }
}

void maker_lines_next(OBSTACLE_LIST& obstacle_, bool clear_){
    if(clear_)line_list.points.clear();
    next_list.header.frame_id = "base_link";//scan->header.frame_id;
    next_list.header.stamp = ros::Time::now();
    for(OBSTACLE_LIST::iterator plist = obstacle_.begin(); plist != obstacle_.end(); plist++){
        p1.x = plist->next_rect.fixed_point[0].x;
        p1.y = plist->next_rect.fixed_point[0].y;
        p1.z = 0;
        p2.x = plist->next_rect.fixed_point[1].x;
        p2.y = plist->next_rect.fixed_point[1].y;
        p2.z = 0;
        p3.x = plist->next_rect.fixed_point[2].x;
        p3.y = plist->next_rect.fixed_point[2].y;
        p3.z = 0;
        p4.x = plist->next_rect.fixed_point[3].x;
        p4.y = plist->next_rect.fixed_point[3].y;
        p4.z = 0;
        next_list.points.push_back(p1);
        next_list.points.push_back(p2);
        next_list.points.push_back(p2);
        next_list.points.push_back(p3);
        next_list.points.push_back(p3);
        next_list.points.push_back(p4);
        next_list.points.push_back(p4);
        next_list.points.push_back(p1);
    }
}

void obstacle_pn_lim(int thres_n, OBSTACLE_LIST& obstacle_){
    for (OBSTACLE_LIST::iterator plist = obstacle_.begin(); plist != obstacle_.end(); ){
        if (plist->scan.number <= thres_n) plist = obstacle_.erase(plist);
        else plist++;
    }
}

void obstacle_combine(OBSTACLE_LIST& obstacle_, OBSTACLE_LIST::iterator& plist1, OBSTACLE_LIST::iterator& plist2){
    plist1->scan.index.splice(plist1->scan.index.end(), plist2->scan.index);
    plist1->scan.number += plist2->scan.number;
    plist2 = obstacle_.erase(plist2);
    rect_ob(plist1,0.30,1.1);
    //rect_ob(plist1,0.01,1);
}

void obstacle_devide(){
}

bool lines_cov(p_point p1, p_point p2,p_point q1, p_point q2){   
    //排斥实验
    bool ret1 = min(p1.x,p2.x) <= max(q1.x,q2.x) &&
                min(q1.x,q2.x) <= max(p1.x,p2.x) &&
                min(p1.y,p2.y) <= max(q1.y,q2.y) &&
                min(q1.y,q2.y) <= max(p1.y,p2.y);
    if(ret1){
        //跨立实验
        // double a = mutiply_cross((p2_x-p1_x),(p2_y-p1_y),(q1_x-p1_x),(q1_y-p1_y));
        // double b = mutiply_cross((p2_x-p1_x),(p2_y-p1_y),(q2_x-p1_x),(q2_y-p1_y));
        // double c = mutiply_cross((q2_x-q1_x),(q2_y-q1_y),(p1_x-q1_x),(p1_y-q1_y));
        // double d = mutiply_cross((q2_x-q1_x),(q2_y-q1_y),(p2_x-q1_x),(p2_y-q1_y));
        double a = mutiply_cross_p(p1, p2, p1, q1);
        double b = mutiply_cross_p(p1, p2, p1, q2);
        double c = mutiply_cross_p(q1, q2, q1, p1);
        double d = mutiply_cross_p(q1, q2, q1, p2);
        bool ret2 = (a*b < 0 && c*d < 0);
        if(ret2) return true;
        else return false;
    }
    else return false;
} 

bool point_in_rect(p_point p, p_rect& rect){
    // double a = mutiply_cross((rect.x[1]-rect.x[0]),(rect.y[1]-rect.y[0]),(x-rect.x[0]),(y-rect.y[0]));
    // double b = mutiply_cross((rect.x[3]-rect.x[2]),(rect.y[3]-rect.y[2]),(x-rect.x[2]),(y-rect.y[2]));
    // double c = mutiply_cross((rect.x[2]-rect.x[1]),(rect.y[2]-rect.y[1]),(x-rect.x[1]),(y-rect.y[1]));
    // double d = mutiply_cross((rect.x[0]-rect.x[3]),(rect.y[0]-rect.y[3]),(x-rect.x[3]),(y-rect.y[3]));
    double a = mutiply_cross_p(rect.fixed_point[0], rect.fixed_point[1], rect.fixed_point[0], p);
    double b = mutiply_cross_p(rect.fixed_point[2], rect.fixed_point[3], rect.fixed_point[2], p);
    double c = mutiply_cross_p(rect.fixed_point[1], rect.fixed_point[2], rect.fixed_point[1], p);
    double d = mutiply_cross_p(rect.fixed_point[3], rect.fixed_point[0], rect.fixed_point[3], p);
    return (a*b >= 0.0 && c*d >= 0.0);
}

bool rect_cov(p_rect& rect1, p_rect& rect2){
    //排斥实验
    double min_x1 = min(min(rect1.fixed_point[0].x,rect1.fixed_point[1].x),min(rect1.fixed_point[2].x,rect1.fixed_point[3].x));
    double max_x1 = max(max(rect1.fixed_point[0].x,rect1.fixed_point[1].x),max(rect1.fixed_point[2].x,rect1.fixed_point[3].x));
    double min_y1 = min(min(rect1.fixed_point[0].y,rect1.fixed_point[1].y),min(rect1.fixed_point[2].y,rect1.fixed_point[3].y));
    double max_y1 = max(max(rect1.fixed_point[0].y,rect1.fixed_point[1].y),max(rect1.fixed_point[2].y,rect1.fixed_point[3].y));
    double min_x2 = min(min(rect2.fixed_point[0].x,rect2.fixed_point[1].x),min(rect2.fixed_point[2].x,rect2.fixed_point[3].x));
    double max_x2 = max(max(rect2.fixed_point[0].x,rect2.fixed_point[1].x),max(rect2.fixed_point[2].x,rect2.fixed_point[3].x));
    double min_y2 = min(min(rect2.fixed_point[0].y,rect2.fixed_point[1].y),min(rect2.fixed_point[2].y,rect2.fixed_point[3].y));
    double max_y2 = max(max(rect2.fixed_point[0].y,rect2.fixed_point[1].y),max(rect2.fixed_point[2].y,rect2.fixed_point[3].y));
    bool ret = (min_x1 <= max_x2) && (min_x2 <= max_x1) && (min_y1 <= max_y2) && (min_y2 <= max_y1);
    if(ret){
        for(size_t i=0; i<4; i++){
            size_t l = (i>=3)?0:(i+1);
            for(size_t j=0; j<4; j++){
                size_t m = (j>=3)?0:(j+1);
                if(lines_cov(rect1.fixed_point[i],rect1.fixed_point[l],rect2.fixed_point[j],rect2.fixed_point[m])) return true;
            }
        }
        if(point_in_rect(rect2.center_point, rect1))return true;
        else if(point_in_rect(rect1.center_point, rect2)) return true;
    }
    else return false;
}

bool cov_zero(OBSTACLE_LIST& obstacle_){
    p_point p;
    p.x = 0;
    p.y = 0;
    for(OBSTACLE_LIST::iterator plist = obstacle_.begin();plist != obstacle_.end();plist++){
       if(point_in_rect(p,plist->rect))return true;
    }
    return false;
}

void pro_point(p_point *p, p_point *p_pro, double dt, double v_x, double v_y, double v_theta){
    double xt = p->x - dt * v_x;
    double yt = p->y - dt * v_y;
    double dtheta = dt * v_theta /180.0 * M_PI;
    p_pro->x = std::cos(-dtheta)*xt-std::sin(-dtheta)*yt;
    p_pro->y = std::sin(-dtheta)*xt+std::cos(-dtheta)*yt;
}

void pro_rect(p_rect& rect, p_rect& rect_pro,double dt, double v_x, double v_y, double v_theta){
    pro_point(&rect.fixed_point[0],&rect_pro.fixed_point[0], dt, v_x, v_y, v_theta);
    pro_point(&rect.fixed_point[1],&rect_pro.fixed_point[1], dt, v_x, v_y, v_theta);
    pro_point(&rect.fixed_point[2],&rect_pro.fixed_point[2], dt, v_x, v_y, v_theta);
    pro_point(&rect.fixed_point[3],&rect_pro.fixed_point[3], dt, v_x, v_y, v_theta);
    pro_point(&rect.center_point,&rect_pro.center_point, dt, v_x, v_y, v_theta);
    pro_point(&rect.match_point,&rect_pro.match_point, dt, v_x, v_y, v_theta);
    pro_point(&rect.match_point_,&rect_pro.match_point_, dt, v_x, v_y, v_theta);
    rect_pro.angle = rect.angle - dt * v_theta;
    rect_pro.angle = rect_pro.angle > 90 ? (rect_pro.angle-180) : (rect_pro.angle < -90 ? (rect_pro.angle + 180) : rect_pro.angle);
    rect_pro.W = rect.W;
    rect_pro.L = rect.L;
}

void pro_obstacle(OBSTACLE_LIST& obstacle_,double dt, double v_x, double v_y, double v_theta){
    if(dt != 0.0 ){
        for(OBSTACLE_LIST::iterator plist = obstacle_.begin();plist != obstacle_.end();plist++){
            pro_rect(plist->rect,plist->pro_rect, dt, v_x, v_y, v_theta);
        }
    }
}

bool obstacle_match(OBSTACLE_LIST::iterator& now_plist, OBSTACLE_LIST& obstacle_){
    if(obstacle_.size()<1){
        now_plist->vel.x = 0;
        now_plist->vel.y = 0;
        now_plist->vel.theta = 0;
        now_plist->vel_match.x = 0;
        now_plist->vel_match.y = 0;
        now_plist->vel_match.theta = 0;
        return false;
    }
    double p_cov, p_l, p_q, p_s, p_angle, p, p_min = 0.0;
    double l_center,l_match;
    int d_theta;
    double s_r = 1.0;
    bool cov, near;
    OBSTACLE_LIST::iterator plist_match = obstacle_.end();
    int index_1 = 0,index_2 = 0;
    for(OBSTACLE_LIST::iterator plist = obstacle_.begin();plist != obstacle_.end();plist++){
        cov = rect_cov(now_plist->rect,plist->pro_rect);
        p_cov = cov ? 1.0 : 0.0;
        near = distance_points(now_plist->rect.center_point,plist->pro_rect.center_point) <= s_r;
        p_l = near ? 1.0 : 0.0;
        p_s = min(now_plist->rect.L,plist->pro_rect.L)/max(now_plist->rect.L,plist->pro_rect.L)*0.5 
              + min(now_plist->rect.W,plist->pro_rect.W)/max(now_plist->rect.W,plist->pro_rect.W)*0.5;
        if(near){
            //plist_match = plist;
            d_theta = now_plist->rect.angle - plist->pro_rect.angle;
            d_theta = d_theta > 90 ? (d_theta - 90) :(d_theta < -90 ? (d_theta + 90) : d_theta);

            p_angle = std::abs(d_theta) > 10 ? 0.0 : (10 - std::abs(d_theta))/10.0;
            
            if((p_cov+p_s+p_angle)/3>0.8 && (p_cov+p_s+p_angle)/3 > p_min){
                plist_match = plist;
                p_min = (p_cov+p_s+p_angle)/3;
            }
        }
    }
    if(plist_match != obstacle_.end()){
        l_match = distance_points(now_plist->rect.match_point,plist_match->rect.match_point);
        if(l_match > (plist_match->pro_rect.W + plist_match->pro_rect.L)/2){
            // if(now_plist->rect.index = 3)index_1 = 0;
            // else index_1 = now_plist->rect.index+1;
            // if(now_plist->rect.index = 0)index_2 = 3;
            // else index_2 = now_plist->rect.index-1;
            // double l1 =  distance_points(now_plist->rect.fixed_point[index_1],plist_match->rect.match_point);
            // double l2 =  distance_points(now_plist->rect.fixed_point[index_2],plist_match->rect.match_point);
            // if(l1>l2)now_plist->rect.match_point = now_plist->rect.fixed_point[index_2];
            // else now_plist->rect.match_point = now_plist->rect.fixed_point[index_1];
            double l1 = distance_points(now_plist->rect.match_point_,plist_match->pro_rect.match_point);
            if(l1 < l_match)now_plist->rect.match_point = now_plist->rect.match_point_;
        }

        l_center = distance_points(now_plist->rect.center_point,plist_match->pro_rect.center_point);
        l_match = distance_points(now_plist->rect.match_point,plist_match->pro_rect.match_point);
        
        double k = l_center / (l_center + l_match);
        k = k  > 0,5 ? 0 : 1;
        now_plist->vel.x = (k*(now_plist->rect.center_point.x - plist_match->pro_rect.center_point.x )
                           + (1-k)*(now_plist->rect.match_point.x - plist_match->pro_rect.match_point.x))
                           / d_scan_time ;
        //now_plist->vel.x = (now_plist->vel.x + plist_match->vel.x)/2;

        now_plist->vel.y = (k*(now_plist->rect.center_point.y - plist_match->pro_rect.center_point.y )
                           + (1-k)*(now_plist->rect.match_point.y - plist_match->pro_rect.match_point.y))
                           / d_scan_time ;
        //now_plist->vel.y = (now_plist->vel.y + plist_match->vel.y)/2;

        now_plist->vel.theta = (d_theta / d_scan_time + plist_match->vel.theta)/2;
        
        //ROS_INFO("x: %f, y: %f, theta: %f",now_plist->vel.x, now_plist->vel.y, now_plist->vel.theta);
        if(std::fabs(now_plist->vel.x - plist_match->vel.x) < 0.1 && std::fabs(now_plist->vel.y - plist_match->vel.y) < 0.1){
            now_plist->vel_match.x = now_plist->vel.x;
            now_plist->vel_match.y = now_plist->vel.y;
            now_plist->vel_match.theta = now_plist->vel.theta;  
        
        }
        else{
            now_plist->vel_match.x = plist_match->vel_match.x;
            now_plist->vel_match.y = plist_match->vel_match.y;
            now_plist->vel_match.theta = plist_match->vel_match.theta;  
        }
        // p1.x = 0;
        // p1.y = 0;
        // p1.z = 0;
        // p2.x = now_plist->rect.center_point.x;
        // p2.y = now_plist->rect.center_point.y;
        // p2.z = 0;
        // line_list.points.push_back(p1);
        // line_list.points.push_back(p2);
        // ROS_INFO("1");
    }
    else {
        now_plist->vel.x = 0;
        now_plist->vel.y = 0;
        now_plist->vel.theta = 0;
        now_plist->vel_match.x = 0;
        now_plist->vel_match.y = 0;
        now_plist->vel_match.theta = 0;        
        return false;
    }
}

void match(OBSTACLE_LIST& obstacle_n,OBSTACLE_LIST& obstacle_l){
    for(OBSTACLE_LIST::iterator plist = obstacle_n.begin();plist != obstacle_n.end();plist++){
        obstacle_match(plist,obstacle_l);
    }
}

void next_obstacle(OBSTACLE_LIST& obstacle_){
    for(OBSTACLE_LIST::iterator plist = obstacle_.begin();plist != obstacle_.end();plist++){
        for(size_t i = 0; i < 4; i++){
            plist->next_rect.fixed_point[i].x =  plist->rect.fixed_point[i].x + plist->vel_match.x * 3;
            plist->next_rect.fixed_point[i].y =  plist->rect.fixed_point[i].y + plist->vel_match.y * 3;
        }
    }
}

double PointToSegDist(p_point p_, p_point q1, p_point q2){
    double cross = (q2.x - q1.x) * (p_.x - q1.x) + (q2.y - q1.y) * (p_.y - q1.y);
    if(cross <= 0) return distance_points(p_,q1);
    
    double d2 = (q2.x - q1.x) * (q2.x - q1.x) + (q2.y - q1.y) * (q2.y - q1.y);
    if (cross >= d2) return distance_points(p_,q2);
    
    double r = cross / d2;
    double px = q1.x + (q2.x - q1.x) * r;
    double py = q1.y + (q2.y - q1.y) * r;
    return std::sqrt((p_.x - px) * (p_.x - px) + (py - p_.y) * (py - p_.y));
}

double PointTORect(p_point p_,p_rect& rect_){
    double min_ = PointToSegDist(p_,rect_.fixed_point[3],rect_.fixed_point[0]);
    double dis;
    for(size_t i = 0; i < 3; i++){
        dis =  PointToSegDist(p_,rect_.fixed_point[i],rect_.fixed_point[i+1]);
        if(min_ > dis){
            min_ = dis;
        }
    }
    return min_;
}

double angle_p(OBSTACLE_LIST& obstacle_,double *dis_,p_rect& rect_self){
    p_point p_zero;
    OBSTACLE_LIST::iterator plist_min = obstacle_.end();
    double dis_min = 25, dis;
    p_zero.x = 0.0;
    p_zero.y = 0.0;
    for(OBSTACLE_LIST::iterator plist = obstacle_.begin();plist != obstacle_.end();plist++){
        if(rect_cov(rect_self,plist->rect)){
            dis = PointTORect(p_zero,plist->rect);
            if(dis<dis_min){
                plist_min = plist;
                dis_min = dis;
            }
        }
    }
    *dis_ = dis_min;
    if(plist_min != obstacle_.end()){
        p1.x = 0;
        p1.y = 0;
        p1.z = 0;
        p2.x = plist_min->rect.center_point.x;
        p2.y = plist_min->rect.center_point.y;
        p2.z = 0;
        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
        if(dis_min>15){
            return 0.0;
        }
        else if(dis_min>7){
            return plist_min->rect.angle*(15 - dis_min)/8.0;
        }
        else {
            return plist_min->rect.angle;
        }
        // p1.x = 0;
        // p1.y = 0;
        // p1.z = 0;
        // p2.x = 1.0*std::cos(plist_min->rect.angle * M_PI / 180.0);
        // p2.y = 1.0*std::sin(plist_min->rect.angle * M_PI / 180.0);
        // p2.z = 0;
    }
    else{
        return 0.0;
        // p1.x = 0;
        // p1.y = 0;
        // p1.z = 0;
        // p2.x = 1;
        // p2.y = 0;
        // p2.z = 0;
    }
    // line_list.points.push_back(p1);
    // line_list.points.push_back(p2);
}

void cluster_obstacle(OBSTACLE_LIST& obstacle_)
{
    OBSTACLE_LIST::iterator plist,plist_;
    bool flag;
    do{
        flag = false;
        for( plist = obstacle_.begin(); plist != obstacle_.end(); plist++)
        {
            plist_ = plist;
            plist_++;
            for(; plist_ != obstacle_.end();)
            {
                if(rect_cov(plist->rect,plist_->rect))
                {
                    obstacle_combine(obstacle_, plist, plist_);
                    flag = true;
                }
                else
                {
                    plist_++;
                } 

            }
        }
    }while(flag == true);
}

void cluster(sensor_msgs::LaserScan& scan_msg, double *p_x, double *p_y){
    line_list.points.clear();
    //next_list.points.clear();
    //fliter_scan(scan_msg,fliter_n,1);
    //cluster_scan(scan_msg,obstacle_now_s,thres_s,min_thres_s,max_thres_s);
    cluster_scan(scan_msg,obstacle_now_l,thres_l,min_thres_l,max_thres_l);
    //obstacle_pn_lim(thres_number,obstacle_now_s);
    obstacle_pn_lim(thres_number,obstacle_now_l);
    //rect_scan(scan_msg, obstacle_now_s, p_x, p_y,0.01,1);
    rect_scan(scan_msg, obstacle_now_l, p_x, p_y,0.30,1.1);
    //cluster_obstacle(obstacle_now_s);
    cluster_obstacle(obstacle_now_l);
    rect_scan(scan_msg, obstacle_now_l, p_x, p_y,0.01,1);
    //pro_obstacle(obstacle_last_s,d_scan_time,speed_x,speed_y,speed_theta);
    //pro_obstacle(obstacle_last_l,d_scan_time,speed_x,speed_y,speed_theta);
    //match(obstacle_now_s,obstacle_last_s);
    //match(obstacle_now_l,obstacle_last_l);

    //next_obstacle(obstacle_now_l);
    if(state_local_start == 1 && state_auto == 1){
        switch(local_state_now){
            case local_stop:
                avoid_x = 0.0;
                avoid_w = 0.0;
                break;
            case local_advance:
                if(!cov_zero(obstacle_now_l)){   
                    p_rect rect_self;
                    rect_self.fixed_point[0].x = 0.0;
                    rect_self.fixed_point[0].y = -1.5;
                    rect_self.fixed_point[1].x = 0.0;
                    rect_self.fixed_point[1].y = 1.5;
                    rect_self.fixed_point[2].x = 15.0;
                    rect_self.fixed_point[2].y = 1.5;
                    rect_self.fixed_point[3].x = 15.0;
                    rect_self.fixed_point[4].y = -1.5;
                    rect_rot(rect_self, (-global_direction));
                    double dis_min =25;
                    double gg = angle_p(obstacle_now_l,&dis_min,rect_self);
                    avoid_w = - std::round(gg/5.0*10.0) / 10.0;
                    if(dis_min >= 15){
                        avoid_w = global_direction/5.0;
                    }
                    avoid_w = avoid_w < -10.0 ? -10.0 : (avoid_w >10.0 ? 10.0 : avoid_w);
                    if(std::fabs(avoid_w) > 8.0f){
                        avoid_x = vel_l;
                    }
                    else {
                        if(dis_min > 12)avoid_x = vel_h;
                        else if(dis_min > 1.0 )avoid_x = ((dis_min-1)*(vel_h - vel_l)/11.0) + vel_l;
                        else avoid_x = vel_l;
                    }
                    ROS_INFO("%f",dis_min); 
                }
                else{
                    //loading
                }
                break;
            case local_rotate:
                avoid_x = 0.0;
                avoid_w = global_direction > 0 ? 10 : -10;
                break;
        }

    }
}

class SubscribeAndPublish{
public:
  SubscribeAndPublish(){
    pub_ob = n.advertise<visualization_msgs::Marker>("lines", 10);
    pub_next_ob = n.advertise<visualization_msgs::Marker>("lines_next", 10);
    pub_scan_pro = n.advertise<sensor_msgs::LaserScan>("scan_pro", 1000);
    pub_cmd = n.advertise<boat_msgs::BoatCmd>("auto_cmd",10);
    pub_vel_info = n.advertise<boat_msgs::BoatVel>("boatvel",10);

    sub_scan = n.subscribe("scan", 100, &SubscribeAndPublish::callback, this);
    sub_pose2d = n.subscribe("pose2D", 100, &SubscribeAndPublish::callback_pose, this);
    sub_auto_state = n.subscribe("auto_state", 10, &SubscribeAndPublish::callback_auto, this);
    sub_global_direction = n.subscribe("global_direction", 10, &SubscribeAndPublish::callback_global_direction, this);
    sub_hedao_direction = n.subscribe("hedao_direction", 10, &SubscribeAndPublish::callback_hedao_direction, this);
  }

  void callback(const sensor_msgs::LaserScan::ConstPtr& scan){
    double scan_time_now = scan->header.stamp.toSec();
    double dd = ros::Time::now().toSec() - scan_time_now;
    if (dd > 1) return;
    if(scan_time_last != 0.0){
        d_scan_time = scan_time_now - scan_time_last;
    }
    scan_time_last = scan_time_now;

    scan_pro.header.stamp = ros::Time::now();
    scan_pro.header.frame_id = "base_link";//scan->header.frame_id;
    //ROS_INFO("1");
    int len = scan->ranges.size();
    //double *p_x = new double[len];
    //double *p_y = new double[len];
    scan_pro.angle_min = scan->angle_min + yaw;
    scan_pro.angle_max = scan->angle_max + yaw;
    scan_pro.angle_increment = scan->angle_increment;
    scan_pro.scan_time = scan->scan_time;
    scan_pro.time_increment = scan->time_increment;
    scan_pro.range_min = scan->range_min;
    scan_pro.range_max = scan->range_max;
    //scan_pro.ranges = scan->ranges;
    //scan_pro.intensities = scan->intensities;
    //size_t len = scan->ranges.size();
    if(scan_now.ranges.size() != len){
        scan_now.ranges.resize(len);
        scan_now.intensities.resize(len);
        scan_pro.ranges.resize(len);
        scan_pro.intensities.resize(len);
    }
    if(scan_last.ranges.size() == len){
        if(mode_1 == 1){
            for(size_t i = 0; i < len; i++){
                scan_now.ranges[i] = scan->ranges[i];
                scan_now.intensities[i] = scan->intensities[i]; 
                if(scan_now.intensities[i] != 0.0){
                    if(scan_last.intensities[i] != 0.0)scan_pro.ranges[i] = scan_now.ranges[i]*0.5 + scan_last.ranges[i]*0.5;
                    else scan_pro.ranges[i] = scan_now.ranges[i];
                }
                else scan_pro.ranges[i] = scan_last.ranges[i];                     
            }
        }
        else if(mode_1 == 2){
            for(size_t i = 0; i < len; i++){
                scan_now.ranges[i] = scan->ranges[i];
                scan_now.intensities[i] = scan->intensities[i]; 
                if(scan_now.intensities[i] != 0.0){
                    if(scan_last.intensities[i] != 0.0)scan_pro.ranges[i] = scan_now.ranges[i]*0.5 + scan_last.ranges[i]*0.5;
                    else scan_pro.ranges[i] = scan_last.ranges[i];
                }
                else scan_pro.ranges[i] = scan_now.ranges[i];                    
            }
        }
        else if(mode_1 == 3){
            for(size_t i = 0; i < len; i++){
                scan_now.ranges[i] = scan->ranges[i];
                scan_now.intensities[i] = scan->intensities[i];
                if(scan_now.intensities[i] != 0.0){
                    if(scan_last.intensities[i] != 0.0)scan_pro.ranges[i] = min(scan_now.ranges[i], scan_last.ranges[i]);
                    else scan_pro.ranges[i] = scan_last.ranges[i];
                }
                else scan_pro.ranges[i] = scan_now.ranges[i];                    
            }
        }
        else if(mode_1 == 4){
            for(size_t i = 0; i < len; i++){
                scan_now.ranges[i] = scan->ranges[i];
                scan_now.intensities[i] = scan->intensities[i];
                if(scan_now.intensities[i] != 0.0){
                    if(scan_last.intensities[i] != 0.0)scan_pro.ranges[i] = min(scan_now.ranges[i], scan_last.ranges[i]);
                    else scan_pro.ranges[i] = scan_now.ranges[i];
                }
                else scan_pro.ranges[i] = scan_last.ranges[i];                    
            }
        }        
        else {
            for(size_t i = 0; i < len; i++){
                scan_now.ranges[i] = scan->ranges[i];
                scan_now.intensities[i] = scan->intensities[i];
                scan_pro.ranges[i] = scan_now.ranges[i];
                scan_pro.intensities[i] = scan_now.intensities[i];
            }                
        }
    }
    else{
        for(size_t i = 0; i < len; i++){
            scan_now.ranges[i] = scan->ranges[i];
            scan_now.intensities[i] = scan->intensities[i];
            scan_pro.ranges[i] = scan_now.ranges[i];
            scan_pro.intensities[i] = scan_now.intensities[i];
        }
    }
    fliter_scan(scan_pro,fliter_n,1);
    scan2points(scan_pro, p_x, p_y, len);
    cluster(scan_pro, p_x, p_y);
    obstacle_last_s = obstacle_now_s;
    obstacle_last_l = obstacle_now_l;
    p_now2last(p_x,p_x_last,len);
    p_now2last(p_y,p_y_last,len);

    if(scan_last.ranges.size() != len){
        scan_last.ranges.resize(len);
        scan_last.intensities.resize(len);
    }
    for(size_t i = 0; i < len; i++){
        scan_last.ranges[i] = scan->ranges[i];
        scan_last.intensities[i] = scan->intensities[i];        
    }

    if(display_s && display_l){
        maker_lines(obstacle_now_s,false);
        maker_lines(obstacle_now_l,false);
        pub_ob.publish(line_list);
    }
    else if(display_s && !display_l){
        maker_lines(obstacle_now_s,false);
        //maker_lines(obstacle_now,false);
        pub_ob.publish(line_list);
    }
    else if(!display_s && display_l){
        //maker_lines(obstacle_now_s,true);
        maker_lines(obstacle_now_l,false);
        pub_ob.publish(line_list);
    }

    if(state_auto == 1){
        boat_msgs::BoatCmd cmd;
        cmd.cmd = boat_msgs::BoatCmd::SPEED;
        cmd.data_float32.push_back(avoid_x);
        cmd.data_float32.push_back(avoid_w);
        pub_cmd.publish(cmd);
    }

    // if(!cov_zero(obstacle_now_l))
    // {
    //     maker_lines_next(obstacle_now_l,false);
    //     pub_next_ob.publish(next_list);
    // }
    //if(obstacle_last.size() > 1)
    //{
    //    ROS_INFO("%d,%d",obstacle_now.begin()->scan.start_index,obstacle_last.begin()->scan.start_index);
    //}
    //delete [] p_x;
    //delete [] p_y;

    //pub_scan_pro.publish(scan_pro);
    //ROS_INFO("2");
  }

  void callback_pose(const geometry_msgs::Pose2D::ConstPtr& pose){
    //ROS_INFO("3");
    static double last_theta = 0, last_x = 0, last_y = 0;
    static double last_time = 0.0;
    static int i=0,k=0;
    static double theta_list[10], x_list[10], y_list[10];
    double now_time= ros::Time::now().toSec();
    double pose_x = pose->x;
    double pose_y = pose->y;
    double pose_theta = pose->theta;
    if(last_time != 0.0){
        if(i >= 10){
            i =0;
        }
        if(k < 10){
            k++;
        }
        double d = (pose_theta - last_theta)/M_PI*180.0;
        double dx = pose_x - last_x;
        double dy = pose_y - last_y;
        double dx_ = std::cos(-last_theta)*dx-std::sin(-last_theta)*dy;
        double dy_ = std::sin(-last_theta)*dx+std::cos(-last_theta)*dy;

        d = d > 180.0 ? (d - 360.0) : (d < -180 ? (d + 360) : d);
        theta_list[i] = d / (now_time - last_time);
        x_list[i] = dx_ / (now_time - last_time);
        y_list[i] = dy_ / (now_time - last_time);
        i++;
        double sum_theta = 0.0, sum_x = 0.0, sum_y = 0.0;
        for(size_t j=0;j<k;j++){
            sum_theta += theta_list[j];
            sum_x += x_list[j];
            sum_y += y_list[j];
        }
        speed_theta = sum_theta/k;
        speed_x = sum_x/k;
        speed_y = sum_y/k;
    }
    //ROS_INFO("%f, theta: %f, x: %f, y: %f", now_time, speed_theta, speed_x, speed_y);
    last_theta = pose_theta;
    last_x = pose_x;
    last_y = pose_y;
    last_time = now_time;
    boat_msgs::BoatVel vel_info;
    vel_info.w = - speed_theta;
    pub_vel_info.publish(vel_info);
  }

  void callback_auto(const std_msgs::String::ConstPtr& state){
      if(state->data == "1"){
          state_auto = 1;
          //ROS_INFO("state = 1");
      }
      else{
          state_auto = 0;
          //ROS_INFO("state = 0");
      }
  }

  void callback_global_direction(const std_msgs::Float32::ConstPtr& direction){
    float temp_direction = direction->data;

    if(std::fabs(temp_direction) <= 180.0){
        global_direction = temp_direction;
        state_local_start = 1;
        state_local_stop = 0;
        switch(local_state_now){
            case local_stop:
                if(std::fabs(global_direction) >= 50){
                    local_state_now = local_rotate;
                }
                else if(std::fabs(global_direction) <= 50){
                    local_state_now = local_advance;
                }
                break;
            case local_advance:
                if(std::fabs(global_direction) >= 50){
                    local_state_now = local_rotate;
                }
                break;
            case local_rotate:
                if(std::fabs(global_direction) <= 40){
                    local_state_now = local_advance;
                }
                break;
        }
    }
    else{
        global_direction = 0.0;
        state_local_start = 0;
        state_local_stop = 1;
        local_state_now = local_stop;
    }
  }

  void callback_hedao_direction(const std_msgs::Float32::ConstPtr& direction){
      global_direction = direction->data;
  }

private:
  ros::NodeHandle n; 
  ros::Publisher pub_ob;
  ros::Publisher pub_next_ob;
  ros::Publisher pub_scan_pro;
  ros::Publisher pub_cmd;
  ros::Publisher pub_vel_info;

  ros::Subscriber sub_scan;
  ros::Subscriber sub_pose2d;
  ros::Subscriber sub_auto_state;
  ros::Subscriber sub_global_direction;
  ros::Subscriber sub_hedao_direction;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "cluster");
    ros::NodeHandle nh_private("~");
    nh_private.param<int>("fliter_n", fliter_n, 3);
    nh_private.param<int>("thres_s", thres_s, 25);
    nh_private.param<int>("thres_l", thres_l, 50);
    nh_private.param<int>("thres_number", thres_number, 1);
    nh_private.param<double>("min_thres_s", min_thres_s, 0.125);
    nh_private.param<double>("max_thres_s", max_thres_s, 0.5);
    nh_private.param<double>("min_thres_l", min_thres_l, 0.25);
    nh_private.param<double>("max_thres_l", max_thres_l, 1.5);
    nh_private.param<bool>("display_s", display_s, false);
    nh_private.param<bool>("display_l", display_l, true);
    nh_private.param<int>("mode_1", mode_1, 1);    
    tf::TransformListener listener;
    tf::StampedTransform transform;
    while (ros::ok()){
        try{
            listener.waitForTransform("/base_link", "/laser", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/base_link", "/laser", ros::Time(0), transform);
            break;
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }
    tf::Matrix3x3 m(transform.getRotation());
    m.getRPY(roll,pitch,yaw);
    ROS_INFO("base_link_to laser [roll: %f, pitch: %f, yaw: %f]",(float)roll,(float)pitch,(float)yaw/M_PI*180);

    next_list.ns = line_list.ns = "lines";
    next_list.action = line_list.action = visualization_msgs::Marker::ADD;
    next_list.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    next_list.id = 1;
    line_list.id = 0;
    next_list.type = line_list.type = visualization_msgs::Marker::LINE_LIST;
    next_list.scale.x = line_list.scale.x = 0.03;
    next_list.color.r = line_list.color.r = 1.0;
    next_list.color.a = line_list.color.a = 1.0;

    SubscribeAndPublish SAPObject;
    ros::MultiThreadedSpinner spinner(4); // Use 2 threads
    spinner.spin();
    return 0;
}