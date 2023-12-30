#include <ros/ros.h>
#include "dynamixel_handler.hpp"

using namespace dyn_x;

bool DynamixelHandler::TmpTest(){
    // dyn_comm.Read(), dyn_comm.SyncRead()の結果が一致するかテストする．
    // 一致しない場合は，dyn_comm.SyncRead()のバグが疑われる．

    auto pos = dyn_comm_.SyncRead(present_position, {2, 3});
    auto pos2 = dyn_comm_.Read(present_position, 2);
    auto pos3 = dyn_comm_.Read(present_position, 3);
    ROS_INFO("pos[2] = %d, pos2 = %d", (int)pos[2], (int)pos2);
    ROS_INFO("pos[3] = %d, pos3 = %d", (int)pos[3], (int)pos3);

    auto vel = dyn_comm_.SyncRead(present_velocity, {2, 3});
    auto vel2 = dyn_comm_.Read(present_velocity, 2);
    auto vel3 = dyn_comm_.Read(present_velocity, 3);
    ROS_INFO("vel[2] = %d, vel2 = %d", (int)vel[2], (int)vel2);
    ROS_INFO("vel[3] = %d, vel3 = %d", (int)vel[3], (int)vel3);

    auto pos_vel = dyn_comm_.SyncRead({present_position, present_velocity}, {2, 3});
    auto pos_vel2 = dyn_comm_.Read({present_position, present_velocity}, 2);
    auto pos_vel3 = dyn_comm_.Read({present_position, present_velocity}, 3);
    ROS_INFO("pos pos_vel[2] = %d, pos_vel2 = %d", (int)pos_vel[2][0], (int)pos_vel2[0]);
    ROS_INFO("vel pos_vel[2] = %d, pos_vel2 = %d", (int)pos_vel[2][1], (int)pos_vel2[1]);
    ROS_INFO("pos pos_vel[3] = %d, pos_vel3 = %d", (int)pos_vel[3][0], (int)pos_vel3[0]);
    ROS_INFO("vel pos_vel[3] = %d, pos_vel3 = %d", (int)pos_vel[3][1], (int)pos_vel3[1]);

    // 以下はSyncReadの代わりにSyncRead_fastを使う場合のテスト
    auto pos_fast = dyn_comm_.SyncRead_fast(present_position, {2, 3});
    auto pos2_fast = dyn_comm_.Read(present_position, 2);
    auto pos3_fast = dyn_comm_.Read(present_position, 3);
    ROS_INFO("pos_fast[2] = %d, pos2_fast = %d", (int)pos_fast[2], (int)pos2_fast);
    ROS_INFO("pos_fast[3] = %d, pos3_fast = %d", (int)pos_fast[3], (int)pos3_fast);

    auto vel_fast = dyn_comm_.SyncRead_fast(present_velocity, {2, 3});
    auto vel2_fast = dyn_comm_.Read(present_velocity, 2);
    auto vel3_fast = dyn_comm_.Read(present_velocity, 3);
    ROS_INFO("vel_fast[2] = %d, vel2_fast = %d", (int)vel_fast[2], (int)vel2_fast);
    ROS_INFO("vel_fast[3] = %d, vel3_fast = %d", (int)vel_fast[3], (int)vel3_fast);

    auto pos_vel_fast = dyn_comm_.SyncRead_fast({present_position, present_velocity}, {2, 3});
    auto pos_vel2_fast = dyn_comm_.Read({present_position, present_velocity}, 2);
    auto pos_vel3_fast = dyn_comm_.Read({present_position, present_velocity}, 3);
    ROS_INFO("pos pos_vel_fast[2] = %d, pos_vel2_fast = %d", (int)pos_vel_fast[2][0], (int)pos_vel2_fast[0]);
    ROS_INFO("vel pos_vel_fast[2] = %d, pos_vel2_fast = %d", (int)pos_vel_fast[2][1], (int)pos_vel2_fast[1]);
    ROS_INFO("pos pos_vel_fast[3] = %d, pos_vel3_fast = %d", (int)pos_vel_fast[3][0], (int)pos_vel3_fast[0]);
    ROS_INFO("vel pos_vel_fast[3] = %d, pos_vel3_fast = %d", (int)pos_vel_fast[3][1], (int)pos_vel3_fast[1]);

    return false;
}

bool DynamixelHandler::Initialize(){
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    // Subscriber / Publisherの設定
    sub_cmd_free_   = nh.subscribe("/dynamixel/cmd_free",   10, DynamixelHandler::CallBackDxlCommandFree);
    sub_cmd_option_ = nh.subscribe("/dynamixel/cmd/option", 10, DynamixelHandler::CallBackDxlCommand_Option);
    sub_cmd_x_pos_  = nh.subscribe("/dynamixel/cmd/x/position", 10, DynamixelHandler::CallBackDxlCommand_X_Position);
    sub_cmd_x_vel_  = nh.subscribe("/dynamixel/cmd/x/velocity", 10, DynamixelHandler::CallBackDxlCommand_X_Velocity);
    sub_cmd_x_cur_  = nh.subscribe("/dynamixel/cmd/x/current",  10, DynamixelHandler::CallBackDxlCommand_X_Current);
    sub_cmd_x_cpos_ = nh.subscribe("/dynamixel/cmd/x/current_position",  10, DynamixelHandler::CallBackDxlCommand_X_CurrentPosition);
    sub_cmd_x_epos_ = nh.subscribe("/dynamixel/cmd/x/extended_position", 10, DynamixelHandler::CallBackDxlCommand_X_ExtendedPosition);

    pub_state_free_   = nh.advertise<dynamixel_handler::DynamixelStateFree>("/dynamixel/state_free", 10);
    pub_state_        = nh.advertise<dynamixel_handler::DynamixelState>("/dynamixel/state", 10);
    pub_error_        = nh.advertise<dynamixel_handler::DynamixelError>("/dynamixel/error", 10);
    pub_config_limit_ = nh.advertise<dynamixel_handler::DynamixelConfig_Limit>("/dynamixel/config/limit", 10);
    pub_config_gain_  = nh.advertise<dynamixel_handler::DynamixelConfig_Gain>("/dynamixel/config/gain", 10);
    pub_config_mode_  = nh.advertise<dynamixel_handler::DynamixelConfig_Mode>("/dynamixel/config/mode", 10);

    // 通信の開始
    int BAUDRATE; string DEVICE_NAME;
    if (!nh_p.getParam("BAUDRATE",         BAUDRATE   ) ) BAUDRATE    =  1000000;
    if (!nh_p.getParam("DEVICE_NAME",      DEVICE_NAME) ) DEVICE_NAME = "/dev/ttyUSB0";
    dyn_comm_ = DynamixelComunicator(DEVICE_NAME.c_str(), BAUDRATE);
    if ( !dyn_comm_.OpenPort() ) {
        ROS_ERROR("Failed to open USB device [%s]", dyn_comm_.port_name().c_str()); 
        return false;
    }

    // serial通信のvarbose設定
    bool serial_varbose;
    if (!nh_p.getParam("dyn_comm_varbose", serial_varbose)) serial_varbose = false;
    dyn_comm_.set_varbose(serial_varbose);

    // serial通信のretry設定
    int num_try, msec_interval;
    if (!nh_p.getParam("dyn_comm_retry_num",     num_try      )) num_try       = 5;
    if (!nh_p.getParam("dyn_comm_inerval_msec",  msec_interval)) msec_interval = 10;
    dyn_comm_.set_retry_config(num_try, msec_interval);
    // return TmpTest();

    // main loop の設定
    if (!nh_p.getParam("loop_rate",        loop_rate_ )) loop_rate_ =  50;
    if (!nh_p.getParam("ratio_state_read",  ratio_state_pub_ )) ratio_state_pub_  =  1;
    if (!nh_p.getParam("ratio_config_read", ratio_config_pub_)) ratio_config_pub_ =  0;
    if (!nh_p.getParam("ratio_error_read",  ratio_error_pub_ )) ratio_error_pub_  =  100;
    if (!nh_p.getParam("use_slipt_write", use_slipt_write_)) use_slipt_write_   =  false;
    if (!nh_p.getParam("use_slipt_read",  use_slipt_read_ )) use_slipt_read_   =  false;
    if (!nh_p.getParam("use_fast_read",   use_fast_read_  )) use_fast_read_    =  true;    
    if (!nh_p.getParam("varbose_write_cmd", varbose_write_cmd_ )) varbose_write_cmd_ =  false;
    if (!nh_p.getParam("varbose_write_cfg", varbose_write_cfg_ )) varbose_write_cfg_ =  false;
    if (!nh_p.getParam("varbose_read_st",     varbose_read_st_     )) varbose_read_st_     =  false;
    if (!nh_p.getParam("varbose_read_st_err", varbose_read_st_err_ )) varbose_read_st_err_ =  false;
    if (!nh_p.getParam("varbose_read_hwerr",  varbose_read_hwerr_  )) varbose_read_hwerr_  =  false;
    if (!nh_p.getParam("varbose_read_cfg",    varbose_read_cfg_    )) varbose_read_cfg_    =  false;
    if (!nh_p.getParam("varbose_callback", varbose_callback_ )) varbose_callback_  =  false;
    if (!nh_p.getParam("varbose_mainloop", varbose_mainloop_ )) varbose_mainloop_  =  false;
    bool tmp = false; !nh_p.getParam("varbose_mainloop", tmp ); varbose_mainloop_  += tmp; // varbose_mainloop_をintでもboolでも受け取れるようにする

    // id_listの作成
    int num_expexted, id_max; 
    if (!nh_p.getParam("init_expected_servo_num",       num_expexted  )) num_expexted  = 0; // 0のときはチェックしない
    if (!nh_p.getParam("init_auto_search_max_id",       id_max        )) id_max        = 35;
    auto num_found = ScanDynamixels(id_max);
    if( num_found==0 ) { // 見つからなかった場合は初期化失敗で終了
        ROS_ERROR("Dynamixel is not found in USB device [%s]", dyn_comm_.port_name().c_str());
        return false;
    }
    if( num_expexted>0 && num_expexted!=num_found ) { // 期待数が設定されているときに、見つかった数が期待数と異なる場合は初期化失敗で終了
        ROS_ERROR("Number of Dynamixel is not matched. Expected [%d], but found [%d]", num_expexted, num_found);
        return false;
    }

    // サーボの初期化
    bool do_clean_hwerr, do_torque_on;
    if (!nh_p.getParam("init_hardware_error_auto_clean",do_clean_hwerr)) do_clean_hwerr= true;
    if (!nh_p.getParam("init_torque_auto_enable",       do_torque_on  )) do_torque_on= true;
    for (auto id : id_list_) if (series_[id] == SERIES_X) {
        if ( do_clean_hwerr ) ClearHardwareError(id, TORQUE_DISABLE);
        ChangeOperatingMode(id, OPERATING_MODE_EXTENDED_POSITION, TORQUE_DISABLE);
        if ( do_torque_on ) TorqueEnable(id);
    }

    // cmd_values_の内部の情報の初期化
    for (auto id : id_list_) if (series_[id] == SERIES_X) { // Xシリーズのみ
        cmd_values_[id][GOAL_PWM]             = goal_pwm.pulse2val            (dyn_comm_.tryRead(goal_pwm            , id), model_[id]);    // エラー時は0
        cmd_values_[id][GOAL_CURRENT]         = goal_current.pulse2val        (dyn_comm_.tryRead(goal_current        , id), model_[id]);    // エラー時は0
        cmd_values_[id][GOAL_VELOCITY]        = goal_velocity.pulse2val       (dyn_comm_.tryRead(goal_velocity       , id), model_[id]);    // エラー時は0
        cmd_values_[id][PROFILE_ACCELERATION] = profile_acceleration.pulse2val(dyn_comm_.tryRead(profile_acceleration, id), model_[id]);    // エラー時は0
        cmd_values_[id][PROFILE_VELOCITY]     = profile_velocity.pulse2val    (dyn_comm_.tryRead(profile_velocity    , id), model_[id]);    // エラー時は0
        cmd_values_[id][GOAL_POSITION]        = goal_position.pulse2val       (dyn_comm_.tryRead(goal_position       , id), model_[id]);    // エラー時は0
    }

    //  readする情報の設定
    list_read_state_ = {
        PRESENT_PWM          ,
        PRESENT_CURRENT      ,
        PRESENT_VELOCITY     ,
        PRESENT_POSITION     ,
        VELOCITY_TRAJECTORY  ,
        POSITION_TRAJECTORY  ,
        PRESENT_INPUT_VOLTAGE,
        PRESENT_TEMPERTURE   ,
    };

    return true;
}

using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

void DynamixelHandler::MainLoop(){
    static int cnt = -1; cnt++;
    static ros::Rate rate(loop_rate_);

    //* デバック
    static float rtime = 0.0, wtime = 0.0, suc_read_part=0.0, suc_read_full=0.0, num_st_read=0.001;
    if ( varbose_mainloop_ !=0 ) 
    if ( cnt % varbose_mainloop_ == 0) {
        ROS_INFO("Loop [%d]: read=%.2f ms, write=%.2f ms, success=%0.1f%%(%0.1f%%)",
            cnt, rtime/varbose_mainloop_, wtime/varbose_mainloop_, 100*suc_read_part/num_st_read, 100*suc_read_full/num_st_read);
        rtime = 0.0; wtime = 0.0; suc_read_part=0.0; suc_read_full=0.0, num_st_read=0.001;
    }
        
    /*　処理時間時間の計測 */ auto rstart = system_clock::now();

    // //* Dynamixelから状態Read & topicをPublish
    static bool is_st_suc = false;
    if ( ratio_state_pub_!=0 ) 
    if ( !is_st_suc || cnt % ratio_state_pub_ == 0 ) { //直前が失敗している場合 or ratio_state_pub_の割合で実行
        is_st_suc = false;
        if ( !use_slipt_read_ ) 
            is_st_suc  = SyncReadStateValues(list_read_state_);
        else for (auto each_state : list_read_state_) 
            is_st_suc += SyncReadStateValues(each_state);
        num_st_read++;
        suc_read_part += is_st_suc;
        suc_read_full += !(is_timeout_read_state_||has_comm_error_read_state_);
        if (is_st_suc) BroadcastDxlState();
    }
    if ( ratio_error_pub_!=0 )
    if ( has_any_hardware_error_ && cnt % ratio_error_pub_ == 0 ) { // 直前が失敗している場合 or ratio_error_pub_の割合で実行
        bool is_err_suc = SyncReadHardwareError();
        if (is_err_suc) BroadcastDxlError();
    }
    if ( ratio_config_pub_!=0 )
    if ( cnt % ratio_config_pub_ == 0 ) { // ratio_config_pub_の割合で実行
        bool is_mode_suc = SyncReadConfigParameter_Mode();
        if (is_mode_suc) BroadcastDxlConfig_Mode();
        bool is_lim_suc = SyncReadConfigParameter_Limit();
        if (is_lim_suc) BroadcastDxlConfig_Limit();
        bool is_gain_suc = SyncReadConfigParameter_Gain();
        if (is_gain_suc) BroadcastDxlConfig_Gain();
    }

    /*　処理時間時間の計測 */ rtime += duration_cast<microseconds>(system_clock::now()-rstart).count() / 1000.0;

    //* topicをSubscribe & Dynamixelへ目標角をWrite
    /* SubscribeDynamixelCmd */ros::spinOnce(); rate.sleep();

    /*　処理時間時間の計測 */ auto wstart = system_clock::now();

    if ( !use_slipt_write_ ) 
        SyncWriteCmdValues(list_wirte_cmd_);
    else for (auto each_cmd : list_wirte_cmd_) 
        SyncWriteCmdValues(each_cmd); 
    is_cmd_updated_.clear();
    list_wirte_cmd_.clear();

    /*　処理時間時間の計測 */ wtime += duration_cast<microseconds>(system_clock::now()-wstart).count() / 1000.0;
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