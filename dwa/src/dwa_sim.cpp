#include "ros/ros.h"
#include "math.h"

using namespace std;

const float DT = 0.05;

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

void CartRobot::outputStatus(){
    cout << "[" << x << "," << y << "," << th << "," << v << "," << o << "]" << endl;
}


int main(int argc, char **argv){
    CartRobot cartbot(0, 0, M_PI/2);
    for(int i=0; i<10; i++){
        cartbot.outputStatus();
        cartbot.updateStatus(1, 0.0);
    }
}
