#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"

using namespace std;

class CartRobot{
    private:
        float x;
        float y;
        float th;
    public:
        CartRobot(float x, float y, float th);
        ~CartRobot();
};

CartRobot::CartRobot(float x_init, float y_init, float th_init){
    x  = x_init;
    y  = y_init;
    th = th_init;
}

CartRobot::~CartRobot(){
    cout << "finished" << endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "coord_robot");
    ros::NodeHandle n;
}
