#include <ros/ros.h>
#include "dynamixel_handler.hpp"

using namespace dyn_x;

bool DynamixelHandler::Initialize(){
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    // Subscriber / Publisherの設定
    sub_cmd_free_  = nh.subscribe("/dynamixel/cmd",   10, DynamixelHandler::CallBackOfDynamixelCommand);
    sub_cmd_x_pos_ = nh.subscribe("/dynamixel/x_cmd/position", 10, DynamixelHandler::CallBackOfDxlCmd_X_Position);
    sub_cmd_x_vel_ = nh.subscribe("/dynamixel/x_cmd/velocity", 10, DynamixelHandler::CallBackOfDxlCmd_X_Velocity);
    sub_cmd_x_cur_ = nh.subscribe("/dynamixel/x_cmd/current",  10, DynamixelHandler::CallBackOfDxlCmd_X_Current);
    sub_cmd_x_cpos_ = nh.subscribe("/dynamixel/x_cmd/current_position",  10, DynamixelHandler::CallBackOfDxlCmd_X_CurrentPosition);
    sub_cmd_x_epos_ = nh.subscribe("/dynamixel/x_cmd/extended_position", 10, DynamixelHandler::CallBackOfDxlCmd_X_ExtendedPosition);

    pub_state_ = nh.advertise<dynamixel_handler::DynamixelState>("/dynamixel/state", 10);

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
    if( num_expexted>0 && num_expexted!=num_found ) {
        ROS_ERROR("Number of Dynamixel is not matched. Expected [%d], but found [%d]", num_expexted, num_found);
        return false;
    }

    // main loop の設定
    if (!nh_p.getParam("varbose",          varbose_   )) varbose_   =  false;
    if (!nh_p.getParam("loop_rate",        loop_rate_ )) loop_rate_ =  50;
    if (!nh_p.getParam("state_read_ratio",  state_pub_ratio_  )) state_pub_ratio_  =  1;
    if (!nh_p.getParam("config_read_ratio", config_pub_ratio_ )) config_pub_ratio_ =  0;
    if (!nh_p.getParam("error_read_ratio",  error_pub_ratio_  )) error_pub_ratio_  =  100;

    //  readする情報の設定
    // todo rosparamで設定できるようにする

    // 内部の情報の初期化
    for (auto id : id_list_) if (series_[id] == SERIES_X) { // Xシリーズのみ
        cmd_values_[id][GOAL_PWM]             = goal_pwm.pulse2val            (dyn_comm_.tryRead(goal_pwm            , id), model_[id]);    // エラー時は0
        cmd_values_[id][GOAL_CURRENT]         = goal_current.pulse2val        (dyn_comm_.tryRead(goal_current        , id), model_[id]);    // エラー時は0
        cmd_values_[id][GOAL_VELOCITY]        = goal_velocity.pulse2val       (dyn_comm_.tryRead(goal_velocity       , id), model_[id]);    // エラー時は0
        cmd_values_[id][PROFILE_ACCELERATION] = profile_acceleration.pulse2val(dyn_comm_.tryRead(profile_acceleration, id), model_[id]);    // エラー時は0
        cmd_values_[id][PROFILE_VELOCITY]     = profile_velocity.pulse2val    (dyn_comm_.tryRead(profile_velocity    , id), model_[id]);    // エラー時は0
        cmd_values_[id][GOAL_POSITION]        = goal_position.pulse2val       (dyn_comm_.tryRead(goal_position       , id), model_[id]);    // エラー時は0
    }

    // サーボの実体としてのDynamixel Chainの初期化, 今回は一旦すべて電流制御付き位置制御モードにしてトルクON    
    for (auto id : id_list_) if (series_[id] == SERIES_X) {
        if ( CheckHardwareError(id) ) ClearHardwareError(id, TORQUE_DISABLE); // ここでは雑に判定している．本来の返り値はuint8_tで各ビットに意味がある. 
        if ( !TorqueDisable(id) ) ROS_WARN("Servo id [%d] failed to disable torque", id);
        if ( !dyn_comm_.tryWrite(operating_mode, id, OPERATING_MODE_EXTENDED_POSITION) ) ROS_WARN("Servo id [%d] failed to set operating mode", id);;  
        if ( !dyn_comm_.tryWrite(profile_acceleration, id, 500)) ROS_WARN("Servo id [%d] failed to set profile_acceleration", id);
        if ( !dyn_comm_.tryWrite(profile_velocity, id, 100)) ROS_WARN("Servo id [%d] failed to set profile_velocity", id);
        if ( !dyn_comm_.tryWrite(homing_offset, id, 0)) ROS_WARN("Servo id [%d] failed to set homing_offset", id);
        if ( !TorqueEnable(id) ) ROS_WARN("Servo id [%d] failed to enable torque", id);
    }

    return true;
}

void DynamixelHandler::MainLoop(){
    static int cnt = 0; cnt++;
    static ros::Rate rate(loop_rate_);

    //* デバック
    // if (varbose_) ShowDynamixelChain();

    //* Dynamixelから状態Read & topicをPublish
    if ( state_pub_ratio_==0 || cnt % state_pub_ratio_ == 0 ) {
        bool is_suc = false;
        if ( !use_slipt_read_ )                      is_suc  = SyncReadStateValues(list_read_state_);
        else for (StateValues st : list_read_state_) is_suc += SyncReadStateValues(st);
        if (is_suc) BroadcastDynamixelState();
    }
    if ( config_pub_ratio_!=0 && cnt % config_pub_ratio_ == 0 ) {
        // const bool is_suc = SyncReadConfigParameter(); 
        // if (is_suc) BroadcastDynamixelConfig();
    }
    if ( error_pub_ratio_!=0  && cnt % error_pub_ratio_ == 0 ) {
        const bool is_suc = SyncReadHardwareError();
        // if (is_suc) = BroadcastDynamixelHardwareError();
    }

    //* topicをSubscribe & Dynamixelへ目標角をWrite
    /* SubscribeDynamixelCmd */ros::spinOnce(); rate.sleep();
    SyncWriteCmdValues(list_wirte_cmd_);
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