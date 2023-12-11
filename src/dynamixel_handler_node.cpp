#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "dynamixel_communicator.h"
#include <dynamixel_handler/DynamixelCmd.h>
#include <dynamixel_handler/DynamixelState.h>

using namespace dyn_x;

std::string DEVICE_NAME;
int         BAUDRATE;
int         loop_rate;
bool        varbose;

struct Dynamixel{
    int32_t goal_position;
    int32_t present_position;
};

DynamixelComunicator dyn_comm;
std::vector<uint8_t> id_list;
std::vector<Dynamixel> dynamixel_chain;
bool is_updated = false;

// ここら変の情報は型番固有の情報なので， dynamixel_parameter.hpp/cpp側に記述して，将来的には自動で読み込ませるようにしたい．
int64_t rad2pulse(double rad) { return rad * 4096.0 / (2.0*M_PI/*rad*/) + 2048; }
double  pulse2rad(int64_t pulse) { return (pulse - 2048 ) * (2.0*M_PI/*rad*/) / 4096.0; }
int64_t mA2pulse(double mA) { return mA / 1.0/*mA*/; }
double  pulse2mA(int64_t pulse) { return pulse * 1.0/*mA*/; }
// int64_t rad_s2pulse(double rad_s) { return rad_s * 0.229/*rev/sec*/ / (2.0*M_PI/*rad*/) /*rad/min*/ * 60 /*rad/sec*/; }
// double  pulse2rad_s(int64_t pulse) { return pulse * 0.229/*rev/sec*/ * (2.0*M_PI/*rad*/) /*rad/min*/ / 60 /*rad/sec*/; }

void FindServo(int id_max) {   
    id_list.clear(); // push_backされれるため， id_listの中身を空にする
    for (int id = 1; id <= id_max; id++) {
        bool is_found = false;
        for (size_t i = 0; i < 5; i++) {
            if (dyn_comm.Ping(id)) is_found = true;
            ros::Duration(0.02).sleep();
        }
        if (is_found) {
            id_list.push_back(id);
            ROS_INFO("Servo id [%d] is found (id range 1 to [%d])", id, id_max);
        }
    }
}

void InitDynamixelChain(int id_max){
    // id_listの作成
    FindServo(id_max);
    assert(id_list.size() != 0);

    // サーボの実体としてのDynamixel Chainの初期化, 今回は一旦すべて位置制御モードにしてトルクON    
    for (auto id : id_list) {
        dyn_comm.Write(id, torque_enable, TORQUE_DISABLE);
        dyn_comm.Write(id, operating_mode, OPERATING_MODE_POSITION);
        dyn_comm.Write(id, profile_acceleration, 500); // 0~32767 数字は適当
        dyn_comm.Write(id, profile_velocity, 100); // 0~32767 数字は適当
        int present_pos = dyn_comm.Read(id, present_position);
        dyn_comm.Write(id, goal_position, present_pos);
        dyn_comm.Write(id, torque_enable, TORQUE_ENABLE);
        if(dyn_comm.Read(id, torque_enable) == TORQUE_DISABLE) ROS_WARN("Servo id [%d] failed to enable torque", id);
    }

    // プログラム内部の変数であるdynamixel_chainの初期化
    dynamixel_chain.resize(id_max);
    std::fill(dynamixel_chain.begin(), dynamixel_chain.end(), Dynamixel{0,0});
    for (auto id : id_list) {
        dynamixel_chain[id].present_position = dyn_comm.Read(id, present_position); // エラー時は0
        dynamixel_chain[id].goal_position    = dyn_comm.Read(id, goal_position);    // エラー時は0
    }
}

void SyncWritePosition(){
    std::vector<int64_t> data_int_list(id_list.size());
    for (size_t i = 0; i < id_list.size(); i++) data_int_list[i] = dynamixel_chain[id_list[i]].goal_position;
    dyn_comm.SyncWrite(id_list, goal_position, data_int_list);
}

bool SyncReadPosition(){
    std::vector<int64_t> data_int_list(id_list.size());
    std::vector<uint8_t> read_id_list(id_list.size());
    for (size_t i = 0; i < id_list.size(); i++) data_int_list[i] = dynamixel_chain[id_list[i]].present_position; // read失敗時に初期化されないままだと危険なので．
    for (size_t i = 0; i < id_list.size(); i++) read_id_list[i]  = 255; // あり得ない値(idは0~252)に設定して，read失敗時に検出できるようにする

    int num_success = dyn_comm.SyncRead_fast(id_list, present_position, data_int_list, read_id_list);
    // エラー処理
    if (num_success != id_list.size()){
        ROS_WARN("SyncReadPosition: %d servo(s) failed to read", (int)(id_list.size() - num_success));
        for (size_t i = 0; i < id_list.size(); i++){
            if (std::find(id_list.begin(), id_list.end(), read_id_list[i]) == id_list.end())
                ROS_WARN("  * servo id [%d] failed to read", id_list[i]);
        }
    }
    // 読み込んだデータをdynamixel_chainに反映
    for (size_t i = 0; i < id_list.size(); i++) // data_int_listの初期値がpresent_positionなので，read失敗時はそのままになる．
        dynamixel_chain[id_list[i]].present_position = data_int_list[i]; 

    return num_success>0 ? true : false; // 1つでも成功したら成功とする.あえて冗長に書いている.
}

void ShowDynamixelChain(){
    // dynamixel_chainのすべての内容を表示
    for (auto id : id_list) {
        ROS_INFO("== Servo id [%d] ==", id);
        ROS_INFO("  present_position [%d] pulse", dynamixel_chain[id].present_position);
        ROS_INFO("  goal_position    [%d] pulse", dynamixel_chain[id].goal_position);
    }
}

void RebootDynamixel(int id){
    dyn_comm.Write(id, torque_enable, TORQUE_DISABLE);
    dyn_comm.Reboot(id);
    ros::Duration(0.5).sleep();
    dyn_comm.Write(id, torque_enable, TORQUE_ENABLE);
    if(dyn_comm.Read(id, torque_enable) == TORQUE_DISABLE) ROS_WARN("Servo id [%d] failed to enable torque", id);
}

void CallBackOfDynamixelCommand(const dynamixel_handler::DynamixelCmd& msg) {
    if (msg.command == "show") ShowDynamixelChain();
    if (msg.command == "init") InitDynamixelChain(dynamixel_chain.size());
    if (msg.command == "reboot") { 
        for (auto id : msg.ids) RebootDynamixel(id);
        if (msg.ids.size() == 0) for (auto id : id_list) RebootDynamixel(id);
    }
    if (msg.command == "write") {
        for (int i = 0; i < msg.ids.size(); i++) {
            int id = msg.ids[i];
            assert(0 <= id && id <= dynamixel_chain.size());
            dynamixel_chain[id].goal_position = rad2pulse(msg.goal_angles[i]);
        }
        is_updated = true;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamixel_handler_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    if (!nh_p.getParam("DEVICE_NAME", DEVICE_NAME)) DEVICE_NAME = "/dev/ttyUSB0";
    if (!nh_p.getParam("BAUDRATE",    BAUDRATE)   ) BAUDRATE    =  1000000;
    if (!nh_p.getParam("loop_rate",   loop_rate)  ) loop_rate   =  50;
    if (!nh_p.getParam("varbose",       varbose)  ) varbose   =  false;
    
    int id_max;    
    if (!nh_p.getParam("dyn_id_max",   id_max)) id_max = 35;
    assert(0 <= id_max && id_max <= 252);
    
    dyn_comm.GetPortHandler(DEVICE_NAME.c_str());
    dyn_comm.set_baudrate(BAUDRATE);
    dyn_comm.OpenPort();

    InitDynamixelChain(id_max);

    ros::Subscriber sub_dyn_cmd   = nh.subscribe("/dynamixel/cmd",   10, CallBackOfDynamixelCommand);
    ros::Publisher  pub_dyn_state = nh.advertise<dynamixel_handler::DynamixelState>("/dynamixel/state", 10);

    ros::Rate rate(loop_rate);
    while(ros::ok()) {
        // Dynamixelから現在角をRead & topicをPublish
        bool is_success = SyncReadPosition();
        if ( is_success ) {
            dynamixel_handler::DynamixelState msg;
            msg.ids.resize(id_list.size());
            msg.present_angles.resize(id_list.size());
            msg.goal_angles.resize(id_list.size());
            for (size_t i = 0; i < id_list.size(); i++) {
                msg.ids[i] = id_list[i];
                msg.present_angles[i] = pulse2rad(dynamixel_chain[id_list[i]].present_position);
                msg.goal_angles[i]    = pulse2rad(dynamixel_chain[id_list[i]].goal_position);
            }
            pub_dyn_state.publish(msg);
        }

        // デバック用
        if (varbose) ShowDynamixelChain();

        // topicをSubscribe & Dynamixelへ目標角をWrite
        ros::spinOnce();
        rate.sleep();
        if( is_updated ) {
            SyncWritePosition();
            is_updated = false;
        } 
    }
}

    // DynamixelのBaudrateを1Mに変更するするときに使った．
    // dyn_comm.Write(6, baudrate_x, BAUDRATE_INDEX_1M);
    // dyn_comm.ClosePort();
    // dyn_comm.set_baudrate(1000000);
    // dyn_comm.OpenPort();


