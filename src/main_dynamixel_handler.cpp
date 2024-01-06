#include <ros/ros.h>
#include "dynamixel_handler.hpp"

using namespace dyn_x;

bool DynamixelHandler::TmpTest(){
    return false;
}

bool DynamixelHandler::Initialize(ros::NodeHandle& nh){
    // Subscriber / Publisherの設定
    sub_command_    = nh.subscribe("/dynamixel/command",    10, DynamixelHandler::CallBackDxlCommand);
    sub_cmd_profile_= nh.subscribe("/dynamixel/cmd/profile", 10, DynamixelHandler::CallBackDxlCmd_Profile);
    sub_cmd_x_pos_  = nh.subscribe("/dynamixel/cmd/x/position", 10, DynamixelHandler::CallBackDxlCmd_X_Position);
    sub_cmd_x_vel_  = nh.subscribe("/dynamixel/cmd/x/velocity", 10, DynamixelHandler::CallBackDxlCmd_X_Velocity);
    sub_cmd_x_cur_  = nh.subscribe("/dynamixel/cmd/x/current",  10, DynamixelHandler::CallBackDxlCmd_X_Current);
    sub_cmd_x_cpos_ = nh.subscribe("/dynamixel/cmd/x/current_position",  10, DynamixelHandler::CallBackDxlCmd_X_CurrentPosition);
    sub_cmd_x_epos_ = nh.subscribe("/dynamixel/cmd/x/extended_position", 10, DynamixelHandler::CallBackDxlCmd_X_ExtendedPosition);
    sub_opt_gain_ = nh.subscribe("/dynamixel/opt/gain/w", 10, DynamixelHandler::CallBackDxlOpt_Gain);
    sub_opt_mode_ = nh.subscribe("/dynamixel/opt/mode/w", 10, DynamixelHandler::CallBackDxlOpt_Mode);
    sub_opt_limit_= nh.subscribe("/dynamixel/opt/limit/w",10, DynamixelHandler::CallBackDxlOpt_Limit);

    pub_state_     = nh.advertise<dynamixel_handler::DynamixelState>("/dynamixel/state", 10);
    pub_error_     = nh.advertise<dynamixel_handler::DynamixelError>("/dynamixel/error", 10);
    pub_opt_limit_ = nh.advertise<dynamixel_handler::DynamixelOption_Limit>("/dynamixel/opt/limit/r", 10);
    pub_opt_gain_  = nh.advertise<dynamixel_handler::DynamixelOption_Gain>("/dynamixel/opt/gain/r", 10);
    pub_opt_mode_  = nh.advertise<dynamixel_handler::DynamixelOption_Mode>("/dynamixel/opt/mode/r", 10);
    pub_opt_goal_  = nh.advertise<dynamixel_handler::DynamixelOption_Goal>("/dynamixel/opt/goal/r", 10);

    ros::NodeHandle nh_p("~");

    // 通信の開始
    int baudrate, latency_timer; string device_name;
    if (!nh_p.getParam("baudrate",         baudrate   ) ) baudrate    =  57600;
    if (!nh_p.getParam("device_name",      device_name) ) device_name = "/dev/ttyUSB0";
    if (!nh_p.getParam("latency_timer",    latency_timer) ) latency_timer = 16;
    dyn_comm_ = DynamixelCommunicator(device_name.c_str(), baudrate, latency_timer);
    if ( !dyn_comm_.OpenPort() ) {
        ROS_ERROR("Failed to open USB device [%s]", dyn_comm_.port_name().c_str()); 
        return false;
    }

    // serial通信のvarbose設定
    bool serial_varbose;
    if (!nh_p.getParam("dyn_comm/varbose", serial_varbose)) serial_varbose = false;
    dyn_comm_.set_varbose(serial_varbose);

    // serial通信のretry設定
    int num_try, msec_interval;
    if (!nh_p.getParam("dyn_comm/retry_num",     num_try      )) num_try       = 5;
    if (!nh_p.getParam("dyn_comm/inerval_msec",  msec_interval)) msec_interval = 10;
    dyn_comm_.set_retry_config(num_try, msec_interval);
    // return TmpTest();

    // main loop の設定
    if (!nh_p.getParam("loop_rate",        loop_rate_ ))        loop_rate_ =  50;
    if (!nh_p.getParam("ratio/state_read",  ratio_state_pub_ )) ratio_state_pub_  =  1;
    if (!nh_p.getParam("ratio/option_read", ratio_option_pub_)) ratio_option_pub_ =  0;
    if (!nh_p.getParam("ratio/error_read",  ratio_error_pub_ )) ratio_error_pub_  =  100;
    if (!nh_p.getParam("ratio/varbose_loop", ratio_mainloop_ )) ratio_mainloop_   =  100;
    if (!nh_p.getParam("max_log_width",      width_log_      )) width_log_        = 7;
    if (!nh_p.getParam("use/split_write", use_split_write_)) use_split_write_ =  false;
    if (!nh_p.getParam("use/split_read",  use_split_read_ )) use_split_read_  =  false;
    if (!nh_p.getParam("use/fast_read",   use_fast_read_  )) use_fast_read_   =  true;
    if (!nh_p.getParam("varbose/callback",     varbose_callback_  )) varbose_callback_  =  false;
    if (!nh_p.getParam("varbose/write_commad", varbose_write_cmd_ )) varbose_write_cmd_ =  false;
    if (!nh_p.getParam("varbose/write_option", varbose_write_opt_ )) varbose_write_opt_ =  false;
    if (!nh_p.getParam("varbose/read_state/raw", varbose_read_st_     )) varbose_read_st_     =  false;
    if (!nh_p.getParam("varbose/read_state/err", varbose_read_st_err_ )) varbose_read_st_err_ =  false;
    if (!nh_p.getParam("varbose/read_option/raw",varbose_read_opt_    )) varbose_read_opt_    =  false;
    if (!nh_p.getParam("varbose/read_option/err",varbose_read_opt_err_)) varbose_read_opt_err_=  false;
    if (!nh_p.getParam("varbose/read_hardware_error",varbose_read_hwerr_  )) varbose_read_hwerr_  =  false;


    // id_listの作成
    int num_expexted, id_max; 
    if (!nh_p.getParam("init/expected_servo_num",       num_expexted  )) num_expexted  = 0; // 0のときはチェックしない
    if (!nh_p.getParam("init/auto_search_max_id",       id_max        )) id_max        = 35;
    auto num_found = ScanDynamixels(id_max);
    if( num_found==0 ) { // 見つからなかった場合は初期化失敗で終了
        ROS_ERROR("Dynamixel is not found in USB device [%s]", dyn_comm_.port_name().c_str());
        return false;
    }
    if( num_expexted>0 && num_expexted>num_found ) { // 期待数が設定されているときに、見つかった数が期待数と異なる場合は初期化失敗で終了
        ROS_ERROR("Number of Dynamixel is not matched. Expected [%d], but found [%d]", num_expexted, num_found);
        return false;
    }

    // 状態のreadの前にやるべき初期化
    for (auto id : id_list_) if (series_[id] == SERIES_X) {
        WriteBusWatchdog(id, 0);
        WriteHomingOffset(id, 0.0); // 設定ファイルからとってこれるようにする
        WriteProfileAcc(id, 600.0*DEG); //  設定ファイルからとってこれるようにする
        WriteProfileVel(id,  60.0*DEG); //  設定ファイルからとってこれるようにする
    }

    // 最初の一回は全ての情報をread & publish
    ROS_INFO("Reading present dynamixel state  ...");
    while ( ros::ok() && SyncReadStateValues()    < 1.0-1e-6 ) rsleep(0.05); BroadcastDxlState();
    while ( ros::ok() && SyncReadHardwareErrors() < 1.0-1e-6 ) rsleep(0.05); BroadcastDxlError();
    while ( ros::ok() && SyncReadOption_Limit() < 1.0-1e-6 ) rsleep(0.05); BroadcastDxlOpt_Limit();
    while ( ros::ok() && SyncReadOption_Gain()  < 1.0-1e-6 ) rsleep(0.05); BroadcastDxlOpt_Gain();
    while ( ros::ok() && SyncReadOption_Mode()  < 1.0-1e-6 ) rsleep(0.05); BroadcastDxlOpt_Mode();
    // while ( ros::ok() && SyncReadOption_Config()  < 1.0-1e-6 ) rsleep(0.05); BroadcastDxlOpt_Config();
    // while ( ros::ok() && SyncReadOption_Extra()  < 1.0-1e-6 ) rsleep(0.05); BroadcastDxlOpt_Extra();

    // cmd_values_の内部の情報の初期化, cmd_values_はreadする関数を持ってないので以下の様に手動で．
    for (auto id : id_list_) if (series_[id] == SERIES_X) { // Xシリーズのみ
        op_mode_[id] = ReadOperatingMode(id);
        cmd_values_[id][GOAL_PWM]      = ReadGoalPWM(id);
        cmd_values_[id][GOAL_CURRENT]  = ReadGoalCurrent(id);
        cmd_values_[id][GOAL_VELOCITY] = ReadGoalVelocity(id);
        cmd_values_[id][PROFILE_ACC]   = ReadProfileAcc(id);
        cmd_values_[id][PROFILE_VEL]   = ReadProfileVel(id);
        cmd_values_[id][GOAL_POSITION] = ReadGoalPosition(id);
    }

    // 状態のreadの後にやるべき初期化
    bool do_clean_hwerr, do_torque_on;
    if (!nh_p.getParam("init/hardware_error_auto_clean",do_clean_hwerr)) do_clean_hwerr= true;
    if (!nh_p.getParam("init/torque_auto_enable",       do_torque_on  )) do_torque_on  = true;
    ROS_INFO("Initializing dynamixel state  ...");
    for (auto id : id_list_) if (series_[id] == SERIES_X) {
        if ( do_clean_hwerr ) ClearHardwareError(id); // 現在の状態を変えない
        if ( do_torque_on )   TorqueOn(id);           // 現在の状態を変えない
    }

    //  readする情報の設定
    bool read_pwm, read_cur, read_vel, read_pos, read_vel_traj, read_pos_traj, read_volt, read_temp;
    if (!nh_p.getParam("read/present_pwm",          read_pwm))      read_pwm = false;
    if (!nh_p.getParam("read/present_current",      read_cur))      read_cur = true;
    if (!nh_p.getParam("read/present_velocity",     read_vel))      read_vel = true;
    if (!nh_p.getParam("read/present_position",     read_pos))      read_pos = true;
    if (!nh_p.getParam("read/velocity_trajectory",  read_vel_traj)) read_vel_traj = false;
    if (!nh_p.getParam("read/position_trajectory",  read_pos_traj)) read_pos_traj = false;
    if (!nh_p.getParam("read/present_input_voltage",read_volt))     read_volt = false;
    if (!nh_p.getParam("read/present_temperature",  read_temp))     read_temp = false;
    if ( read_pwm ) list_read_state_.insert(PRESENT_PWM);
    if ( read_cur ) list_read_state_.insert(PRESENT_CURRENT);
    if ( read_vel ) list_read_state_.insert(PRESENT_VELOCITY);
    if ( read_pos ) list_read_state_.insert(PRESENT_POSITION);
    if ( read_vel_traj ) list_read_state_.insert(VELOCITY_TRAJECTORY);
    if ( read_pos_traj ) list_read_state_.insert(POSITION_TRAJECTORY);
    if ( read_volt ) list_read_state_.insert(PRESENT_INPUT_VOLTAGE);
    if ( read_temp ) list_read_state_.insert(PRESENT_TEMPERTURE);

    return true;
}

using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

void DynamixelHandler::MainLoop(const ros::TimerEvent& e){
    static int cnt = -1; cnt++;
    static float rtime=0, wtime=0, num_st_suc_p=1, num_st_suc_f=1, num_st_read=1;

/* 処理時間時間の計測 */ auto wstart = system_clock::now();
    //* topicをSubscribe & Dynamixelへ目標角をWrite
    SyncWriteCommandValues();
    // rsleep(0.0002); // 0.2ms待つ // 無くてもよさそう
/* 処理時間時間の計測 */ wtime += duration_cast<microseconds>(system_clock::now()-wstart).count() / 1000.0;

    //* Dynamixelから状態Read & topicをPublish
    static double rate_suc_st = 0.0;
    if ( ratio_state_pub_!=0 ) 
    if ( rate_suc_st<1.0 || cnt % ratio_state_pub_ == 0 ) { //直前が失敗している場合 or ratio_state_pub_の割合で実行
/* 処理時間時間の計測 */ auto rstart = system_clock::now();
        rate_suc_st = SyncReadStateValues();
        num_st_read++;
        num_st_suc_p += rate_suc_st > 0.0;
        num_st_suc_f += rate_suc_st > 1.0-1e-6;
        if ( rate_suc_st>0.0 ) BroadcastDxlState();
/* 処理時間時間の計測 */ rtime += duration_cast<microseconds>(system_clock::now()-rstart).count() / 1000.0;
    }
    if ( ratio_error_pub_!=0 )
    if ( cnt % ratio_error_pub_ == 0 ) { // ratio_error_pub_の割合で実行
        double rate_suc_err = SyncReadHardwareErrors();
        if ( rate_suc_err>0.0) BroadcastDxlError();
    }
    if ( ratio_option_pub_!=0 )
    if ( cnt % ratio_option_pub_ == 0 ) { // ratio_option_pub_の割合で実行
        double rate_suc_lim = SyncReadOption_Limit(); // 処理を追加する可能性を考えて，変数を別で用意する冗長な書き方をしている．
        if ( rate_suc_lim >0.0 ) BroadcastDxlOpt_Limit();
        double rate_suc_gain = SyncReadOption_Gain();
        if ( rate_suc_gain>0.0 ) BroadcastDxlOpt_Gain();
        double rate_suc_mode = SyncReadOption_Mode();
        if ( rate_suc_mode>0.0 ) BroadcastDxlOpt_Mode();
        double rate_suc_goal = SyncReadOption_Goal();
        if ( rate_suc_goal>0.0 ) BroadcastDxlOpt_Goal();
    }

    //* デバック
    if ( ratio_mainloop_ !=0 ) 
    if ( cnt % ratio_mainloop_ == 0) {
        float partial_suc = 100*num_st_suc_p/num_st_read; float full_suc = 100*num_st_suc_f/num_st_read;
        char msg[100]; sprintf(msg, "Loop [%d]: write=%.2fms read=%.2fms(p/f=%3.0f%%/%3.0f%%)",
                                cnt, wtime/ratio_mainloop_, rtime/num_st_read, partial_suc, full_suc);
        if (full_suc < 80) ROS_ERROR("%s", msg); else if (partial_suc < 99) ROS_WARN("%s", msg); else ROS_INFO("%s", msg);
        /* mainloopで行われてる処理の計測時間を初期化 */wtime = 0.0; 
    }
    if ( cnt % max({loop_rate_, ratio_mainloop_, 10}) == 0) // stateのreadの周期で行われてる処理の初期化
        rtime = num_st_suc_p = num_st_suc_f = num_st_read=0.00001;
        
}

void DynamixelHandler::Terminate(int sig){
    ROS_INFO("Terminating DynamixelHandler ...");
    ros::NodeHandle nh_p("~");
    bool do_torque_off;
    if (!nh_p.getParam("term/torque_auto_disable", do_torque_off  )) do_torque_off  = true;
    if ( do_torque_off ) for ( auto id : id_list_ ) if (series_[id] == SERIES_X) TorqueOff(id);
    StopDynamixels();
    ROS_INFO("Terminating DynamixelHandler Finished");
    ros::shutdown();
}

#include <signal.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamixel_handler_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh; // DynaxeimlHandlerのStaticメンバとして宣言すると，ros::init()の前に呼ばれちゃうみたいなので，ここで宣言する．
    if ( !DynamixelHandler::Initialize(nh) ) {
        ROS_ERROR("Failed to initialize DynamixelHandler");
        return 0;
    }
    signal(SIGINT, DynamixelHandler::Terminate);

    // ros::Timer timer = nh.createTimerを使って書き換え
    ros::Timer timer = nh.createTimer(
        ros::Duration(1.0/DynamixelHandler::loop_rate_), 
        DynamixelHandler::MainLoop
    );
    ros::spin();
}