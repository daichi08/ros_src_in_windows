#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "cmath"

#define DEG2RAD(x) ((x)*M_PI/180.0)
#define DT 0.05

using namespace std;
vector<float> current_status(5);
bool          subscribe_flg = false;

class SimRobot{
    private:
        float V_MAX;
        float V_MIN;
        float OM_MAX;
        float OM_MIN;
        float V_ACC_MAX;
        float OM_ACC_MAX;
    public:
        SimRobot();
        vector<float> set_limits();
};

SimRobot::SimRobot(){
    V_MAX      = 1.4;
    V_MIN      = 0.0;
    V_ACC_MAX  = 9*DT;
    OM_MAX     = DEG2RAD(60);
    OM_MIN     = DEG2RAD(-60);
    OM_ACC_MAX = DEG2RAD(100)*DT;
}

vector<float> SimRobot::set_limits(){
    vector<float> limits(4);
    limits[0] = min(current_status[3]+V_ACC_MAX,  V_MAX);
    limits[1] = max(current_status[3]-V_ACC_MAX,  V_MIN);
    limits[2] = min(current_status[4]+OM_ACC_MAX, OM_MAX);
    limits[3] = max(current_status[4]-OM_ACC_MAX, OM_MIN);

    return limits;
}



class DWA{
    private:
        SimRobot simbot;
        float    V_RES;
        float    OM_RES;
        float    SIM_TIME;
        float    SAMP_TIME;
        int      PRE_STEP;
        float    W_ANG;
        float    W_VEL;
        float    W_OBS;
    public:
        DWA();
        int predict_status();
};

DWA::DWA(){
    V_RES  = 0.1;
    OM_RES = DEG2RAD(2);

    SIM_TIME  = 2;
    SAMP_TIME = 0.1;
    PRE_STEP  = int(SIM_TIME/SAMP_TIME);

    W_ANG = 0.5;
    W_VEL = 0.5;
    W_OBS = 0.5;
}

int DWA::predict_status(){
    vector<float> limits = simbot.set_limits();

    int v_steps  = int((limits[0] - limits[1])/V_RES);
    int om_steps = int((limits[2] - limits[3])/OM_RES);
    int index = 0;

    float v  = 0;
    float om = 0;
    vector<vector<vector<float>>> next_statuses(v_steps*om_steps, vector<vector<float>>(PRE_STEP, current_status));

    for(int i = 0; i < v_steps; i++){
        v = limits[1]+V_RES*i;
        for(int j = 0; j < om_steps; j++){
            om = limits[3]+OM_RES*j;

            for(int k = 0; k < PRE_STEP; k++){
                if(k == 0){
                    next_statuses[index][k][0] = v * cos(current_status[2]) * SAMP_TIME + current_status[0];
                    next_statuses[index][k][1] = v * sin(current_status[2]) * SAMP_TIME + current_status[1];
                    next_statuses[index][k][2] = om * SAMP_TIME + current_status[2];
                }else{
                    next_statuses[index][k][0] = v * cos(next_statuses[index][k-1][2]) * SAMP_TIME + next_statuses[index][k-1][0];
                    next_statuses[index][k][1] = v * sin(next_statuses[index][k-1][2]) * SAMP_TIME + next_statuses[index][k-1][1];
                    next_statuses[index][k][2] = om * SAMP_TIME + next_statuses[index][k-1][2];
                }
                next_statuses[index][k][3] = v;
                next_statuses[index][k][4] = om;
            }
            index++;
        }
    }

    for(const auto next_status:next_statuses){
        for(const auto status_vector:next_status){

        }
    }

    /*for(int i=0;i < v_steps*om_steps; i++){
        for(int j=0;j < PRE_STEP;j++){
            cout << "[";
            for(int k=0;k < 5; k++){
                cout << next_statuses[i][j][k] << ",";
            }
            cout << "]" << endl;
        }
        cout << "===================" << endl;
    }
    cout << "*******************" << endl;
    */

    return 0;
}

void callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    current_status = msg->data;
    subscribe_flg  = true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "dwa_sim");
    ros::NodeHandle n;
    ros::Subscriber sub_status = n.subscribe("cart_status", 5, callback);
    ros::Rate       rate(20);

    DWA simulator;

    while(ros::ok()){
        if(subscribe_flg){
            simulator.predict_status();
        }
        ros::spinOnce();
        rate.sleep();
    }
}
