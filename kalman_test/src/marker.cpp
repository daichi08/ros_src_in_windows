#include "ros/ros.h"
#include "vector"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
using namespace std;

vector< vector< vector<float> > > obstacles;

void callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    int   datasize  = msg->ranges.size();
    float rad_inc   = msg->angle_increment;
    float rad_min   = msg->angle_min;

    vector<float>           before_point(2);
    vector<float>           current_point(2);
    vector< vector<float> > linear_points;

    // 初期化
    obstacles.clear();
    before_point[0] = msg->ranges[0]*cos(msg->angle_min);
    before_point[1] = msg->ranges[0]*sin(msg->angle_min);

    for(int i = 0; i < datasize; i++){
        float angle      = rad_min + rad_inc * i;
        float range      = msg->ranges[i];
        float point_x    = range * cos(angle);
        float point_y    = range * sin(angle);
        float norm       = 0.0;
        float similarity = 0.0;

        current_point[0] = point_x;
        current_point[1] = point_y;
        if (isnan(range)){
            before_point = current_point;
        }else{
            norm = sqrt(
                        pow(current_point[0]-before_point[0], 2) +
                        pow(current_point[1]-before_point[1], 2)
                    );
            similarity = 1/(1+norm);
            if(norm > 0.98){
                linear_points.push_back(current_point);
            }else{
                obstacles.push_back(linear_points);
                linear_points.clear();
            }
            before_point = current_point;
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "marker_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("scan", 1, callback);
    ros::Rate rate(20);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();

        if(obstacles.empty()){
            ROS_WARN("nothing data");
        }else{
            for(const auto& line : obstacles){
                for(const auto& point : line){
                    ROS_INFO("[%f, %f]", point[0], point[1]);
                }
            }
        }
    }
}
