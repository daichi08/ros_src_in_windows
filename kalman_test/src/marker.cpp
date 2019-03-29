#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "marker_node");
    printf("hello w");
}
