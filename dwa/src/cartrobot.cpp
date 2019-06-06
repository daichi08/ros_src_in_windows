#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"

#define DT 0.05

using namespace std;
float u_v  = 0;
float u_om = 0;

class CartRobot{
    private:
        vector<float> status_vector;
    public:
        CartRobot();
        ~CartRobot();

        void update_status(float u_v, float u_om);
        vector<float> publish_status();
};

CartRobot::CartRobot(){
    status_vector.resize(5);
    status_vector = {0, 0, M_PI/2, 0, 0};
}

CartRobot::~CartRobot(){
    cout << "finished" << endl;
}

void CartRobot::update_status(float u_v, float u_om){
    status_vector[3] = u_v;
    status_vector[4] = u_om;
}

vector<float> CartRobot::publish_status(){
    return status_vector;
}

void callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    u_v  = msg->data[0];
    u_om = msg->data[1];
}

int main(int argc, char **argv){
    ros::init(argc, argv, "cartrobot");
    ros::NodeHandle n;
    ros::Publisher  pub = n.advertise<std_msgs::Float32MultiArray>("cart_status",5);
    ros::Subscriber sub = n.subscribe("inputs", 10, callback);
    ros::Rate       rate(20);

    CartRobot cartbot;
    std_msgs::Float32MultiArray statuses;
    statuses.data.resize(5);

    while(ros::ok()){
        statuses.data = cartbot.publish_status();
        pub.publish(statuses);

        ros::spinOnce();
        rate.sleep();

        cartbot.update_status(u_v, u_om);
    }
}
