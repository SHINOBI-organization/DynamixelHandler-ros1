#include <ros/ros.h>
#include "dynamixel_handler.hpp"

bool DynamixelHandler::Initialize(){
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    // Subscriber / Publisherの設定
    sub_cmd_ = nh.subscribe("/dynamixel/cmd",   10, DynamixelHandler::CallBackOfDynamixelCommand);
    pub_dyn_state_ = nh.advertise<dynamixel_handler::DynamixelState>("/dynamixel/state", 10);

    // 通信の開始
    int BAUDRATE; string DEVICE_NAME;
    if (!nh_p.getParam("BAUDRATE",         BAUDRATE   ) ) BAUDRATE    =  1000000;
    if (!nh_p.getParam("DEVICE_NAME",      DEVICE_NAME) ) DEVICE_NAME = "/dev/ttyUSB0";
    dyn_comm_ = DynamixelComunicator(DEVICE_NAME.c_str(), BAUDRATE);
    if ( !dyn_comm_.OpenPort() ) {
        ROS_ERROR("Failed to open USB device [%s]", dyn_comm_.port_name().c_str()); 
        return false;
    }

    // serial通信のretry設定
    int num_try, msec_interval;
    if (!nh_p.getParam("dyn_comm_retry_num",     num_try      )) num_try       = 5;
    if (!nh_p.getParam("dyn_comm_inerval_msec",  msec_interval)) msec_interval = 10;
    dyn_comm_.set_retry_config(num_try, msec_interval);

    // id_listの作成
    int num_expexted, id_max;
    if (!nh_p.getParam("dyn_num_chained_servo",    num_expexted)) num_expexted = 0; // 0のときはチェックしない
    if (!nh_p.getParam("dyn_search_max_id",        id_max      )) id_max       = 35;
    ROS_INFO("Auto scanning Dynamixel (id range 1 to [%d])", id_max);
    auto num_found = ScanDynamixels(id_max);
    if( num_found==0 ) {
        ROS_ERROR("Dynamixel is not found in USB device [%s]", dyn_comm_.port_name().c_str());
        return false;
    }
    if( num_expexted>0 && num_expexted>num_found ) {
        ROS_ERROR("Number of Dynamixel is not matched. Expected [%d], but found [%d]", num_expexted, num_found);
        return false;
    }

    // main loop の設定
    if (!nh_p.getParam("varbose",          varbose_    )) varbose_     =  false;
    if (!nh_p.getParam("loop_rate",        loop_rate_  )) loop_rate_   =  50;
    if (!nh_p.getParam("error_read_ratio", error_ratio_)) error_ratio_ =  100;

    // プログラム内部の変数であるdynamixel_chainの初期化
    for (auto id : id_list_x_) {
        dynamixel_chain[id].present_position = dyn_comm_.Read(present_position, id); // エラー時は0
        dynamixel_chain[id].goal_position    = dyn_comm_.Read(goal_position, id);    // エラー時は0
    }

    // サーボの実体としてのDynamixel Chainの初期化, 今回は一旦すべて電流制御付き位置制御モードにしてトルクON    
    for (auto id : id_list_x_) {
        if ( TorqueDisable(id) ) ROS_WARN("Servo id [%d] failed to disable torque", id);
        // 以下は適当，あとでちゃんと書く
        dyn_comm_.tryWrite(operating_mode, id, OPERATING_MODE_EXTENDED_POSITION);  
        dyn_comm_.tryWrite(profile_acceleration, id, 500); // 0~32767 数字は適当
        dyn_comm_.tryWrite(profile_velocity, id, 100); // 0~32767 数字は適当
        dyn_comm_.tryWrite(homing_offset, id, 0);
        if ( CheckHardwareError(id) ) ClearHardwareError(id, TORQUE_DISABLE); // ここでは雑に判定している．本来の返り値はuint8_tで各ビットに意味がある. 
        if ( TorqueEnable(id) ) ROS_WARN("Servo id [%d] failed to enable torque", id);
    }
    return true;
}

void DynamixelHandler::MainLoop(){
    static int cnt = 0;
    static ros::Rate rate(loop_rate_);

    if ( ++cnt % error_ratio_ == 0 ) SyncReadHardwareError();

    // Dynamixelから現在角をRead & topicをPublish
    bool is_success = SyncReadPosition();
    if ( is_success ) {
        dynamixel_handler::DynamixelState msg;
        msg.ids.resize(id_list_x_.size());
        msg.present_angles.resize(id_list_x_.size());
        msg.goal_angles.resize(id_list_x_.size());
        for (size_t i = 0; i < id_list_x_.size(); i++) {
            msg.ids[i] = id_list_x_[i];
            msg.present_angles[i] = pulse2rad(dynamixel_chain[id_list_x_[i]].present_position);
            msg.goal_angles[i]    = pulse2rad(dynamixel_chain[id_list_x_[i]].goal_position);
        }
        pub_dyn_state_.publish(msg);
    }

    // デバック用
    if (varbose_) ShowDynamixelChain();

    // topicをSubscribe & Dynamixelへ目標角をWrite
    ros::spinOnce();
    rate.sleep();
    // if( is_updated ) {
        SyncWritePosition();
        is_updated = false;
    // } 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamixel_handler_node");

    if ( !DynamixelHandler::Initialize() ) {
        ROS_ERROR("Failed to initialize DynamixelHandler");
        return 0;
    }

    while(ros::ok()) {
        DynamixelHandler::MainLoop();
    }
}