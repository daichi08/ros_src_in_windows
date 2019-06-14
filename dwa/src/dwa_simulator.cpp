/**
 * @file   dwa_simulator.cpp
 * @brief  DWAで経路計画をおこなうプログラム
 * @author daichi08
 * @date   2019-06-04
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32MultiArray.h"
#include "vector"
#include "cmath"

/**
 * @namespace std
 * @brief     標準のやつ
 */
using namespace std;

/**
 * @def   DEG2RAD
 * @brief deg->rad変換用
 */
#define DEG2RAD(x) ((x)*M_PI/180.0)

//! 点群の分割に使用する類似度
const float SIM_LIMIT = 0.95;
//! LRF座標系とロボット座標系の位相
const float ANGLE_DIFF = M_PI/2;
//! 制御周期の逆数
const float DT = 0.05;
//! 次のステップにおける速度入力
float u_v  = 0;
//! 次のステップにおける回転速度入力
float u_om = 0;
//! LRFからのデータ受信フラグ
bool lrf_sub_flg = false;
//! 分割された点群を収めるベクトル
vector< vector< vector<float> > > objects;
//! 実際のロボットの現在状態
vector<float> current_status(5);

/**
 * @brief 実際のロボットクラス
 */
class CartRobot{
    private:
        vector<float> status_vector;
    public:
        /**
         * @brief 状態ベクトルの初期化
         */
        CartRobot(){
            status_vector.resize(5);
            status_vector = {0, 0, M_PI/2, 0, 0}
        };
        /**
         * @brief 状態ベクトルの更新
         */
        void update_status(){
            status_vector[3] = u_v;
            status_vector[4] = u_om;
        };
        /**
         * @brief 状態ベクトルの配信
         */
        vector<float> current_status(){
            return status_vector;
        };
};

/**
 * @brief シミュレーション用ロボットクラス
 */
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
/**
 * @brief 物理パラメータの設定
 */
SimRobot::SimRobot(){
    V_MAX      = 1.4;
    V_MIN      = 0.0;
    V_ACC_MAX  = 9*DT;
    OM_MAX     = DEG2RAD(60);
    OM_MIN     = DEG2RAD(-60);
    OM_ACC_MAX = DEG2RAD(100)*DT;
}
/**
 * @brief 次のステップで取りうる速度類のリミット計算 
 */
vector<float> SimRobot::set_limits(){
    vector<float> limits(4);
    limits[0] = min(current_status[3]+V_ACC_MAX,  V_MAX);
    limits[1] = max(current_status[3]-V_ACC_MAX,  V_MIN);
    limits[2] = min(current_status[4]+OM_ACC_MAX, OM_MAX);
    limits[3] = max(current_status[4]-OM_ACC_MAX, OM_MIN);

    return limits;
}

/**
 * @brief DWAシミュレーション用クラス
 */
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

/**
 * @brief シミュレーション用パラメータの設定
 */
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

/**
 * @brief 一定時間後に存在できる位置の計算
 * @return まだ適当
 */
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
    return 0;
}

/**
 * @brief LRFから得た点群を分割するコールバック関数
 */
void division_point(const sensor_msgs::LaserScan::ConstPtr& msg){
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

    objects.clear();
    lrf_sub_flg = true;

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
                objects.push_back(linear_points);
                linear_points.clear();
            }else if(similarity > SIM_LIMIT){
                linear_points.push_back(current_point);
            }else if(!linear_points.empty()){
                objects.push_back(linear_points);
                linear_points.clear();
            }
        }
        before_point = current_point;
        index += 1;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "dwa_sim");
    ros::NodeHandle n;
    ros::Subscriber lrf_sub = n.subscribe("scan", 10, division_point);
    ros::Rate       rate(20);

    DWA       simulator;
    CartRobot cartbot;

    while(ros::ok()){
        if(lrf_sub_flg){
            simulator.predict_status();

        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
