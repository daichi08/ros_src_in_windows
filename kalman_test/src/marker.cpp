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
            if(similarity > 0.95){
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
    ros::Publisher m_pub = n.advertise<visualization_msgs::MarkerArray>("markers", 100);
    ros::Rate rate(20);

    while(ros::ok()){
        visualization_msgs::MarkerArray marker_array;
        if(obstacles.empty()){
            ROS_WARN("nothing data");
        }else{
            for(int i = 0; i < obstacles.size(); i++){
                if(!obstacles[i].empty()){
                    visualization_msgs::Marker marker;

                    geometry_msgs::Point first_point, last_point;

                    auto tmp_first_point = obstacles[i].front();
                    auto tmp_last_point  = obstacles[i].back();

                    first_point.x = tmp_first_point[0];
                    first_point.y = tmp_first_point[1];
                    first_point.z = 0;
                    last_point.x  = tmp_last_point[0];
                    last_point.y  = tmp_last_point[1];
                    last_point.z  = 0;

                    marker.header.frame_id = "laser";
                    marker.id = i;
                    marker.ns = "object";
                    marker.type = marker.LINE_LIST;
                    marker.action = marker.ADD;

                    marker.points.push_back(first_point);
                    marker.points.push_back(last_point);

                    marker.color.r = 1;
                    marker.color.g = 0;
                    marker.color.b = 0;
                    marker.color.a = 0.8;

                    marker.scale.x = 0.01;
                    marker.scale.y = 0;
                    marker.scale.z = 0;

                    marker.lifetime = ros::Duration();

                    marker_array.markers.push_back(marker);
                }
            }
        }
        m_pub.publish(marker_array);

        ros::spinOnce();
        rate.sleep();
    }
}

