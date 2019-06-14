#include "ros/ros.h"
#include "vector"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32MultiArray.h"
using namespace std;

vector< vector< vector<float> > > obstacles;
const float SIM_LIMIT  = 0.94;
const float ANGLE_DIFF = M_PI/2;

void callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    int   index      = 0;
    int   datasize   = msg->ranges.size();
    float rad_inc    = msg->angle_increment;
    float rad_min    = msg->angle_min;
    float point_norm = 0.0;
    float similarity = 0.0;
    float angle      = 0.0;

    vector<float>           before_point(2);
    vector<float>           current_point(2);
    vector< vector<float> > linear_points;

    // 初期化
    obstacles.clear();

    for(auto range : msg->ranges){
        angle = rad_min + rad_inc * index + ANGLE_DIFF;
        if(index == 0){
            current_point[0] = range * cos(angle);
            current_point[1] = range * sin(angle);
            if(!isnan(range) && range != 0){
                linear_points.push_back(current_point);
            }
        }else if(!isnan(range) && range != 0){
            current_point[0] = range * cos(angle);
            current_point[1] = range * sin(angle);
            point_norm = sqrt(
                            pow(current_point[0]-before_point[0], 2) +
                            pow(current_point[1]-before_point[1], 2)
                         );
            similarity = 1/(1+point_norm);

            if(index == datasize-1){
                if(similarity > SIM_LIMIT){
                    linear_points.push_back(current_point);
                }
                obstacles.push_back(linear_points);
                linear_points.clear();
            }else if(similarity > SIM_LIMIT){
                linear_points.push_back(current_point);
            }else if(!linear_points.empty()){
                obstacles.push_back(linear_points);
                linear_points.clear();
            }
        }
        before_point = current_point;
        index += 1;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "marker");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("scan", 10, callback);
    ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("array", 10);
    ros::Rate rate(20);

    while(ros::ok()){
        std_msgs::Float32MultiArray pointcloud;
        if(obstacles.empty()){
            ROS_WARN("nothing data");
        }else{
            for(auto obstacle : obstacles){
                cout << "*" << endl;
                for(auto points : obstacle){
                    for(auto pos : points){
                        cout << pos << ",";
                    }
                }
                cout << "*" << endl;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

