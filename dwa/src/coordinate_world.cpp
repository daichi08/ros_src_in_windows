#include "ros/ros.h"
#include "math.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;

const float DT = 0.05;
float u_v = 0.0;
float u_o = 0.0;

class CartRobot{
    private:
        float x;
        float y;
        float th;
        float v;
        float o;
    public:
        CartRobot(float x, float y, float th);
        ~CartRobot();

        void updateStatus(float u_v, float u_o);
        void outputStatus();
        vector<float> publishStatus();
};

CartRobot::CartRobot(float x_init, float y_init, float th_init){
    x  = x_init;
    y  = y_init;
    th = th_init;
    v  = 0.0;
    o  = 0.0;
}

CartRobot::~CartRobot(){
    cout << "system shutdown" << endl;
}

void CartRobot::updateStatus(float u_v, float u_o){
    x  = x + u_v * cos(th) * DT;
    y  = y + u_v * sin(th) * DT;
    th = th + u_o * DT;
    v  = u_v;
    o  = u_o;
}

vector<float> CartRobot::publishStatus(){
    vector<float> status_vector(5);
    status_vector[0] = x;
    status_vector[1] = y;
    status_vector[2] = th;
    status_vector[3] = v;
    status_vector[4] = o;

    return status_vector;
}

void CartRobot::outputStatus(){
    cout << "[" << x << "," << y << "," << th << "," << v << "," << o << "]" << endl;
}

void callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    u_v = msg->data[0];
    u_o = msg->data[1];
}

int main(int argc, char **argv){
    ros::init(argc, argv, "cart_status");
    ros::NodeHandle n;

    ros::Publisher  pub = n.advertise<std_msgs::Float32MultiArray>("cart_status", 10);
    ros::Subscriber sub = n.subscribe("inputs", 10, callback);
    ros::Rate rate(20);

    CartRobot cartbot(0, 0, M_PI/2);
    std_msgs::Float32MultiArray status_vector;

    status_vector.data.resize(5);

    while(ros::ok()){
        u_v = 0.0;
        u_o = 0.0;

        ros::spinOnce();
        rate.sleep();

        cartbot.updateStatus(u_v, u_o);
        //cartbot.outputStatus();

        status_vector.data = cartbot.publishStatus();
        pub.publish(status_vector);
    }
}
