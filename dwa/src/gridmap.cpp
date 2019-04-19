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

vector< vector<float> > gridmap;

vector< vector<int> > calcObsGrid(auto ranges, float range_max, float angle_inc, float angle_min){
    int i       = 0;
    int x_i     = 0;
    int y_i     = 0;
    float x_p   = 0.0;
    float y_p   = 0.0;
    float angle = 0.0;
    vector< vector<int> > obstacle_ids;

    // グリッドマップに対して障害物の生成
    for(const auto range : ranges){
        if(!isnan(range) && range != 0){
            angle = angle_min + angle_inc * i;
            x_p = range * cos(angle);
            y_p = range * sin(angle);

            x_i = int(round((x_p+range_max)/GRID_SIZE));
            y_i = int(round((y_p+range_max)/GRID_SIZE));

            obstacle_ids.push_back({x_i, y_i});
            gridmap[x_i][y_i] = 1;
        }
        i += 1;
    }

    // 重複削除
    sort(obstacle_ids.begin(), obstacle_ids.end());
    obstacle_ids.erase(unique(obstacle_ids.begin(), obstacle_ids.end()), obstacle_ids.end());

    return obstacle_ids;
}

inline int calcGrid(int range_max,float position){
    return int(round((position+range_max)/GRID_SIZE));

}

float correctAtan2(int y, int x){
    float angle = atan2(y, x);
    if(angle<0)
        angle += 2*M_PI;
    return angle;
}

inline int calcAngleID(float angle){
    return int(floor(angle/ANGLE_RES));
}

void precasting(auto obstacle_ids, int origin_grid){
    float dist     = 0.0;
    float angle    = 0.0;
    int angle_id   = 0;
    int relative_x = 0;
    int relative_y = 0;

    vector<int>   angle_ids;
    vector<float> distances;

    for(auto obstacle_id : obstacle_ids){
        relative_x = obstacle_id[0]-origin_grid;
        relative_y = obstacle_id[1]-origin_grid;

        dist     = sqrt(relative_x^2 + relative_y^2);
        angle    = correctAtan2(relative_y, relative_x);
        angle_id = calcAngleID(angle);
        angle_ids.push_back(angle_id);
        distances.push_back(dist);
    }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    /* 宣言 */
    float range_max = msg->range_max;
    float angle_inc = msg->angle_increment;
    float angle_min = msg->angle_min;
    int map_range   = int(ceil(range_max*2/GRID_SIZE));
    int origin_grid = calcGrid(range_max, 0);

    vector< vector<int> > obstacle_ids = calcObsGrid(msg->ranges, range_max, angle_inc, angle_min);
    gridmap.assign(map_range, vector<float>(map_range, 0.0));
    //precasting(obstacle_ids, origin_grid);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "dwa_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("scan", 10, scanCallback);
    ros::Publisher  pub = nh.advertise<std_msgs::Float32MultiArray>("gridmap", 10);
    ros::Rate rate(1);

    while(ros::ok()){
        if(!gridmap.empty()){
            std_msgs::Float32MultiArray grids;
            for(auto grid_list : gridmap){
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

