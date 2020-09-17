#include <cmath>
#include <algorithm>
#include <string>
#include <list>

typedef struct p_point
{
    double x;
    double y;
}p_point;

typedef struct p_scan
{
    std::list<int> index;
    int start_index;
    int stop_index;
    int number;
}p_scan;

typedef struct p_rect
{
    //double x[4];
    //double y[4];
    p_point fixed_point[4];
    p_point center_point;
    double W;
    double L;
    double angle;
    p_point match_point;
    int index;
    p_point match_point_;
}p_rect;

typedef struct p_vel
{
    double x;
    double y;
    double theta;
}p_vel;

typedef struct obstacle
{
    p_scan scan;
    p_rect rect;
    p_rect pro_rect;
    p_rect next_rect;
    p_vel vel;
    p_vel vel_match;
}obstacle;

typedef struct obstacle_cluster
{
    std::list<obstacle> index;
    p_rect rect;
    p_vel vel;
}obstacle_cluster;

typedef std::list<obstacle> OBSTACLE_LIST;
typedef std::list<int> INT_LIST;
typedef std::list<obstacle_cluster> OBSTACLE_CLUSTER_LIST;

enum local_state{
	local_stop=1, local_advance, local_rotate
};
