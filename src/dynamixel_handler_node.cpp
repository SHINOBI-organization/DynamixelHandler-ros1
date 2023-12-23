#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "dynamixel_communicator.h"
#include <dynamixel_handler/DynamixelCmd.h>
#include <dynamixel_handler/DynamixelState.h>

#include <chrono>

using std::vector;
using namespace dyn_x;
using namespace std::chrono_literals;

std::string DEVICE_NAME;
int         BAUDRATE;
int         loop_rate;
bool        varbose;
int         error_ratio;

struct Dynamixel{
    int32_t goal_position;
    int32_t present_position;
};

DynamixelComunicator dyn_comm;
vector<uint8_t> id_list;
vector<Dynamixel> dynamixel_chain;
bool is_updated = false;
bool has_hardware_error = false;

// ここら変の情報は型番固有の情報なので， dynamixel_parameter.hpp/cpp側に記述して，将来的には自動で読み込ませるようにしたい．
// int64_t deg2pulse(double deg) { return deg * 4096.0 / 360.0 + 2048; }
// double  pulse2deg(int64_t pulse) { return (pulse - 2048 ) * 360.0 / 4096.0; }
int64_t rad2pulse(double rad) { return rad * 4096.0 / (2.0 * M_PI) + 2048; }
double  pulse2rad(int64_t pulse) { return (pulse - 2048 ) * 2.0 * M_PI / 4096.0; }
// int64_t mA2pulse(double mA) { return mA / 1.0; }
// double  pulse2mA(int64_t pulse) { return pulse * 1.0; }

void InitDynamixelChain(int id_max){
    // id_listの作成
    ROS_INFO("Auto scanning Dynamixel (id range 1 to [%d])", id_max);
    id_list = dyn_comm.Scan(id_max);
    if(id_list.size() == 0) {
        ROS_ERROR("Dynamicel is not found in USB device [%s]", dyn_comm.port_name().c_str());
        exit(1);
    }

    // サーボの実体としてのDynamixel Chainの初期化, 今回は一旦すべて電流制御付き位置制御モードにしてトルクON    
    for (auto id : id_list) {
        if ( dyn_comm.TorqueDisable(id) ) ROS_WARN("Servo id [%d] failed to disable torque", id);
        dyn_comm.Write(id, torque_enable, TORQUE_DISABLE);
        dyn_comm.Write(id, operating_mode, OPERATING_MODE_EXTENDED_POSITION);  
        dyn_comm.Write(id, profile_acceleration, 500); // 0~32767 数字は適当
        dyn_comm.Write(id, profile_velocity, 100); // 0~32767 数字は適当
        int present_pos = dyn_comm.Read(id, present_position);
        dyn_comm.Write(id, goal_position, present_pos);
        bool is_hardware_error = dyn_comm.Read(id, hardware_error_status); // ここでは雑に判定している．本来の返り値はuint8_tで各ビットに意味がある. 
        if (is_hardware_error) dyn_comm.ClearError(id, TORQUE_DISABLE);
        if ( dyn_comm.TorqueEnable(id) ) ROS_WARN("Servo id [%d] failed to enable torque", id);
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
    vector<int64_t> data_int_list(id_list.size());
    for (size_t i = 0; i < id_list.size(); i++) data_int_list[i] = dynamixel_chain[id_list[i]].goal_position;
    dyn_comm.SyncWrite(id_list, goal_position, data_int_list);
}

bool SyncReadPosition(){
    vector<int64_t> data_int_list(id_list.size());
    vector<uint8_t> read_id_list(id_list.size());
    for (size_t i = 0; i < id_list.size(); i++) data_int_list[i] = dynamixel_chain[id_list[i]].present_position; // read失敗時に初期化されないままだと危険なので．
    for (size_t i = 0; i < id_list.size(); i++) read_id_list[i]  = 255; // あり得ない値(idは0~252)に設定して，read失敗時に検出できるようにする

    int num_success = (has_hardware_error)
                        ? dyn_comm.SyncRead     (id_list, present_position, data_int_list, read_id_list)  // エラーがあるときは遅い方
                        : dyn_comm.SyncRead_fast(id_list, present_position, data_int_list, read_id_list); // エラーがあるときは早い方
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

void SyncReadHardwareError(){
    ROS_INFO("SyncReadHardwareError: Checking hardware error");
    
    vector<int64_t> error_int_list(id_list.size());
    vector<uint8_t> read_id_list(id_list.size());
    for (size_t i = 0; i < id_list.size(); i++) error_int_list[i] = 0;   // read失敗時にエラーだと誤認されないように．
    for (size_t i = 0; i < id_list.size(); i++) read_id_list[i]  = 255; // あり得ない値(idは0~252)に設定して，read失敗時に検出できるようにする

    int num_success = dyn_comm.SyncRead(id_list, hardware_error_status, error_int_list, read_id_list);

    has_hardware_error = false;
    for (auto error : error_int_list) has_hardware_error += (bool)error; // errorがあるときは0以外の値になる．
    if ( !has_hardware_error ) return;

    ROS_ERROR("SyncReadHardwareError: Hardware Error is detected");
    for (int i = 0; i < id_list.size(); i++) {
        uint8_t error = error_int_list[i];
        if ((error >> HARDWARE_ERROR_INPUT_VOLTAGE)     & 0b1 ) ROS_ERROR("  * servo id [%d] : INPUT_VOLTAGE",     id_list[i]);
        if ((error >> HARDWARE_ERROR_MOTOR_HALL_SENSOR) & 0b1 ) ROS_ERROR("  * servo id [%d] : MOTOR_HALL_SENSOR", id_list[i]);
        if ((error >> HARDWARE_ERROR_OVERHEATING)       & 0b1 ) ROS_ERROR("  * servo id [%d] : OVERHEATING",       id_list[i]);
        if ((error >> HARDWARE_ERROR_MOTOR_ENCODER)     & 0b1 ) ROS_ERROR("  * servo id [%d] : MOTOR_ENCODER",     id_list[i]);
        if ((error >> HARDWARE_ERROR_ELECTRONICAL_SHOCK)& 0b1 ) ROS_ERROR("  * servo id [%d] : ELECTRONICAL_SHOCK",id_list[i]);
        if ((error >> HARDWARE_ERROR_OVERLOAD)          & 0b1 ) ROS_ERROR("  * servo id [%d] : OVERLOAD",          id_list[i]);
    }
}

void ShowDynamixelChain(){
    // dynamixel_chainのすべての内容を表示
    for (auto id : id_list) {
        ROS_INFO("== Servo id [%d] ==", id);
        ROS_INFO("  present_position [%d] pulse", dynamixel_chain[id].present_position);
        ROS_INFO("  goal_position    [%d] pulse", dynamixel_chain[id].goal_position);
    }
}

void CallBackOfDynamixelCommand(const dynamixel_handler::DynamixelCmd& msg) {
    if (msg.command == "show") ShowDynamixelChain();
    if (msg.command == "init") InitDynamixelChain(dynamixel_chain.size());
    if (msg.command == "reboot") { 
        for (auto id : msg.ids) dyn_comm.ClearError(id);
        if (msg.ids.size() == 0) for (auto id : id_list) dyn_comm.ClearError(id);
    }
    if (msg.command == "write") {
        if( msg.goal_angles.size() == msg.ids.size()) {
        for (int i = 0; i < msg.ids.size(); i++) {
            int id = msg.ids[i];
            dynamixel_chain[id].goal_position = rad2pulse(msg.goal_angles[i]);
        }
        is_updated = true;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamixel_handler_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    if (!nh_p.getParam("DEVICE_NAME",      DEVICE_NAME)) DEVICE_NAME = "/dev/ttyUSB0";
    if (!nh_p.getParam("BAUDRATE",         BAUDRATE)   ) BAUDRATE    =  1000000;
    if (!nh_p.getParam("loop_rate",        loop_rate)  ) loop_rate   =  50;
    if (!nh_p.getParam("varbose",            varbose)  ) varbose     =  false;
    if (!nh_p.getParam("error_read_ratio", error_ratio)) error_ratio =  100;
    
    int id_max;    
    if (!nh_p.getParam("dyn_id_max",   id_max)) id_max = 35;
    assert(0 <= id_max && id_max <= 252);
    
    dyn_comm.GetPortHandler(DEVICE_NAME.c_str());
    dyn_comm.set_baudrate(BAUDRATE);
    dyn_comm.OpenPort();

    InitDynamixelChain(id_max);

    ros::Subscriber sub_cmd = nh.subscribe("/dynamixel/cmd",   10, CallBackOfDynamixelCommand);
    ros::Publisher  pub_dyn_state = nh.advertise<dynamixel_handler::DynamixelState>("/dynamixel/state", 10);

    ros::Rate rate(loop_rate);
    uint cnt = 0;
    while(ros::ok()) {
        if ( ++cnt % error_ratio == 0 ) { SyncReadHardwareError(); cnt=0; };

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


