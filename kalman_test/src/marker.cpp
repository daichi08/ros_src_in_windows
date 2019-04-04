#include "ros/ros.h"
#include "vector"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
using namespace std;

vector< vector< vector<float> > > obstacles;

void callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    int   index      = 0;
    int   datasize   = msg->ranges.size();
    float rad_inc    = msg->angle_increment;
    float rad_min    = msg->angle_min;
    float norm       = 0.0;
    float similarity = 0.0;
    float angle      = 0.0;
    float point_x    = 0.0;
    float point_y    = 0.0;

    vector<float>           before_point(2);
    vector<float>           current_point(2);
    vector< vector<float> > linear_points;

    // 初期化
    obstacles.clear();
    before_point[0] = msg->ranges[0]*cos(msg->angle_min);
    before_point[1] = msg->ranges[0]*sin(msg->angle_min);

    for(auto range : msg->ranges){
        angle      = rad_min + rad_inc * index;
        point_x    = range * cos(angle);
        point_y    = range * sin(angle);
        norm       = 0.0;
        similarity = 0.0;

        current_point[0] = point_x;
        current_point[1] = point_y;
        if (!isnan(range)){
            norm = sqrt(
                        pow(current_point[0]-before_point[0], 2) +
                        pow(current_point[1]-before_point[1], 2)
                    );
            similarity = 1/(1+norm);

            if(index == datasize-1){
                linear_points.push_back(current_point);
                obstacles.push_back(linear_points);
                linear_points.clear();
            }else if(similarity > 0.95){
                linear_points.push_back(current_point);
            }else{
                obstacles.push_back(linear_points);
                linear_points.clear();
            }
        }
        before_point = current_point;
        index += 1;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "marker_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("scan", 10, callback);
    ros::Publisher m_pub = n.advertise<visualization_msgs::MarkerArray>("markers", 10);
    ros::Rate rate(20);

    int index = 0;

    while(ros::ok()){
        visualization_msgs::MarkerArray marker_array;
        if(obstacles.empty()){
            ROS_WARN("nothing data");
        }else{
            index = 0;
            for(auto obstacle : obstacles){
                if(!obstacle.empty()){
                    visualization_msgs::Marker marker;

                    geometry_msgs::Point first_point, last_point;

                    auto tmp_first_point = obstacle.front();
                    auto tmp_last_point  = obstacle.back();

                    first_point.x = tmp_first_point[0];
                    first_point.y = tmp_first_point[1];
                    first_point.z = 0;
                    last_point.x  = tmp_last_point[0];
                    last_point.y  = tmp_last_point[1];
                    last_point.z  = 0;

                    marker.header.frame_id = "laser";
                    marker.id = index;
                    marker.ns = "object";
                    marker.type = marker.LINE_LIST;
                    marker.action = marker.ADD;

                    marker.points.push_back(first_point);
                    marker.points.push_back(last_point);

                    marker.color.r = 0;
                    marker.color.g = 0;
                    marker.color.b = 1;
                    marker.color.a = 0.8;

                    marker.scale.x = 0.01;
                    marker.scale.y = 0;
                    marker.scale.z = 0;

                    marker.lifetime = ros::Duration(1);

                    marker_array.markers.push_back(marker);
                }
                index += 1;
            }
        }
        m_pub.publish(marker_array);

        ros::spinOnce();
        rate.sleep();
    }
}

