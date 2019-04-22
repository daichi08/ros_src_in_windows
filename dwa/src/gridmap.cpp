/* for ROS */
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32MultiArray.h"

/* for calculate */
#include "vector"
#include "math.h"
#include "iostream"
#include "algorithm"
#include "fstream"
#include "iostream"

using namespace std;

const float GRID_SIZE = 0.1;
const float ANGLE_RES = 1 * M_PI / 180.0;
const float LRF_RANGE = 6;

vector< vector<float> > gridmap;

struct gridDB{
    int   x_i;
    int   y_i;
    float dist;
    int   angle_id;
};

struct obsDB{
    int   x_i;
    int   y_i;
    float dist;
    int   angle_id;
};

inline int calcGrid(int range_max,float position){
    return int(round((position+range_max)/GRID_SIZE));

}

int calcAngleID(int x, int y){
    float angle    = atan2(y, x);
    if(angle<0)
        angle += 2*M_PI;

    int angle_id = int(floor(angle/ANGLE_RES));
    return angle_id;
}

void printGridmap(){
    for(auto grid_list : gridmap){
        for(auto grid : grid_list){
            cout << grid;
        }
        cout << endl;
    }
    cout << "=====================================================================================" << endl;
}

void output4csv(){
    ofstream file;
    file.open("slam.csv", ios::trunc);

    for(auto grid_list:gridmap){
        for(auto grid:grid_list){
            file << grid << ",";
        }
        file << endl;
    }
    file.close();
}

vector<obsDB> calcObsGrid(auto ranges, float range_max, float angle_inc, float angle_min){
    int   i        = 0;
    int   x_i      = 0;
    int   y_i      = 0;
    int   angle_id = 0;
    float x_p      = 0.0;
    float y_p      = 0.0;
    float angle    = 0.0;
    vector<obsDB>  obstacles;

    // グリッドマップに対して障害物の生成
    for(auto range : ranges){
        if(!isnan(range) && range != 0){
            angle = angle_min + angle_inc * i;
            x_p = range * cos(angle);
            y_p = range * sin(angle);

            x_i = calcGrid(range_max, x_p);
            y_i = calcGrid(range_max, y_p);

            gridmap[x_i][y_i] = 1.0;

            gridmap[calcGrid(range_max, 0)][calcGrid(range_max, 0)] = 5;

            if(angle<0){
                angle_id = int(floor((angle+2*M_PI)/ANGLE_RES));
            }else{
                angle_id = int(floor(angle/ANGLE_RES));
            }
            obsDB obstacle = {x_i, y_i, range, angle_id};
            obstacles.push_back(obstacle);
        }
        i += 1;
    }
    return obstacles;
}

vector< vector<gridDB> > precasting(float range_max, int map_range){
    float x_p      = 0.0;
    float y_p      = 0.0;
    float dist     = 0.0;
    int   angle_id = 0;
    vector< vector<gridDB>> precast(int(round(2*M_PI/ANGLE_RES))+1);

    for(int i=0; i<map_range; i++){
        for(int j=0; j<map_range; j++){
            x_p = i * GRID_SIZE - range_max;
            y_p = j * GRID_SIZE - range_max;

            dist     = sqrt(pow(x_p, 2) + pow(y_p, 2));
            angle_id = calcAngleID(x_p, y_p);
            gridDB grid = {i, j, dist, angle_id};

            precast[angle_id].push_back(grid);
        }
    }
    return precast;
}

void reycasting(auto obstacles, auto precast){
    vector<gridDB> grid_list;

    for(auto obstacle:obstacles){
        grid_list = precast[obstacle.angle_id];
        for(auto grid:grid_list){
            if(grid.dist>obstacle.dist){
                gridmap[grid.x_i][grid.y_i] = 2;
            }
        }
    }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    /* 宣言 */
    float range_max = LRF_RANGE;
    float angle_inc = msg->angle_increment;
    float angle_min = msg->angle_min;
    //int map_range   = int(ceil(range_max*2/GRID_SIZE));
    int map_range = int(round(2*LRF_RANGE/GRID_SIZE));

    gridmap.assign(map_range, vector<float>(map_range, 0.0));

    vector< vector<gridDB> > precast   = precasting(range_max, map_range);
    vector<obsDB>            obstacles = calcObsGrid(msg->ranges, range_max, angle_inc, angle_min);
    reycasting(obstacles, precast);

    output4csv();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "dwa_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("scan", 10, scanCallback);
    ros::Publisher  pub = nh.advertise<std_msgs::Float32MultiArray>("gridmap", 10);
    ros::Rate rate(1);

    //while(ros::ok()){
    int i = 0;
    while(i < 2){
        if(!gridmap.empty()){
            //printGridmap();
        }

        ros::spinOnce();
        rate.sleep();
        i += 1;
    }
}

