/* for ROS */
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32MultiArray.h"

/* for calculate */
#include "vector"
#include "math.h"
#include "iostream"
#include "algorithm"

using namespace std;

const float GRID_SIZE = 0.1;
const float ANGLE_RES = 3 * M_PI / 180.0;

vector< vector<float> > grids;
vector< vector<int> >   obstacle_ids;
vector<int>             angle_ids;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    int i         = 0;
    int map_range = 0;
    float angle   = 0.0;
    float x_p     = 0.0; // real x position
    float y_p     = 0.0; // real y position
    int x_i       = 0.0;
    int y_i       = 0.0;
    float angle_inc = msg->angle_increment;
    float angle_min = msg->angle_min;
    float range_max = msg->range_max;
    int zero_i      = int(round(range_max/GRID_SIZE));
    float obstacle_angle = 0.0;
    float grid_angle     = 0.0;
    int angle_id      = 0;
    int grid_angle_id = 0;
    vector<int>::iterator it;

    map_range = int(ceil(range_max*2/GRID_SIZE));
    grids.assign(map_range, vector<float>(map_range, 0.0));

    obstacle_ids.clear();

    // グリッドマップに対して障害物の生成
    for(const auto range : msg->ranges){
        if(!isnan(range) && range != 0){
            angle = angle_min + angle_inc * i;
            x_p = range * cos(angle);
            y_p = range * sin(angle);

            x_i = int(round((x_p+range_max)/GRID_SIZE));
            y_i = int(round((y_p+range_max)/GRID_SIZE));

            obstacle_ids.push_back({x_i, y_i});
            grids[x_i][y_i] = 1;
        }
        i += 1;
    }
    // 重複削除
    sort(obstacle_ids.begin(), obstacle_ids.end());
    obstacle_ids.erase(unique(obstacle_ids.begin(), obstacle_ids.end()), obstacle_ids.end());

    // 計測できない範囲に対して値代入
    grids[zero_i][zero_i] = 9;
    for(auto obstacle_id : obstacle_ids){
        obstacle_angle = atan2(obstacle_id[1]-zero_i, obstacle_id[0]-zero_i);
        if(obstacle_angle < 0){
            obstacle_angle += 2*M_PI;
        }
        angle_id = int(floor(obstacle_angle/ANGLE_RES));
        angle_ids.push_back(angle_id);
    }

    for(int i = 0; i < map_range; i++){
        for(int j = 0; j < map_range; j++){
            grid_angle = atan2(j-zero_i, i-zero_i);
            if(grid_angle < 0){
                grid_angle += 2*M_PI;
            }
            grid_angle_id = int(floor(grid_angle/ANGLE_RES));
            it = find(angle_ids.begin(), angle_ids.end(), grid_angle_id);
            if(it != angle_ids.end()){
                grids[i][j] += 5;
            }
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "dwa_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("scan", 10, scanCallback);
    ros::Publisher  pub = nh.advertise<std_msgs::Float32MultiArray>("gridmap", 10);
    ros::Rate rate(1);

    while(ros::ok()){
        if(!grids.empty()){
            std_msgs::Float32MultiArray gridmap;
            for(auto grid_list : grids){
                for(auto grid : grid_list){
                    cout << grid;
                }
                cout << endl;
            }
            cout << "********************************" << endl;
        }

        ros::spinOnce();
        rate.sleep();
    }
}

