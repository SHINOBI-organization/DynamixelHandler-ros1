#include "dynamixel_handler.hpp"

using namespace dyn_x;

//* 基本機能をまとめた関数たち

// 各シリーズのDynamixelを検出する．
uint8_t DynamixelHandler::ScanDynamixels(uint8_t id_max) {
    ROS_INFO("Auto scanning Dynamixel (id range 1 to [%d])", id_max);
    id_list_.clear();
    for (int id = 0; id <= id_max; id++) {
        if ( !dyn_comm_.tryPing(id) ) continue;
        auto dyn_model = dyn_comm_.tryRead(model_number, id);
        switch ( dynamixel_series(dyn_model) ) { 
            case SERIES_X: ROS_INFO(" * X series servo id [%d] is found", id);
                model_[id] = dyn_model;
                series_[id] = SERIES_X;
                id_list_.push_back(id); break;
            case SERIES_P: ROS_INFO(" * P series servo id [%d] is found", id);
                model_[id] = dyn_model;
                series_[id] = SERIES_P;
                id_list_.push_back(id); break;
            default: ROS_WARN(" * Unkwon model [%d] servo id [%d] is found", (int)dyn_model, id);
        }
    }
    ROS_INFO("Finish scanning Dynamixel");
    return id_list_.size();
}

// 全てのモータの動作を停止させる．
void DynamixelHandler::StopDynamixels(){
    vector<uint8_t> id_list; 
    for (auto id : id_list_) if ( series_[id]==SERIES_X ) id_list.push_back(id);
    vector<int64_t> offset_pulse(id_list.size(), 0);
    dyn_comm_.SyncWrite(homing_offset ,id_list, offset_pulse); // マジで謎だが，BusWatchdogを設定するとHomingOffset分だけ回転してしまう...多分ファームrウェアのバグ
    vector<int64_t> bus_watchtime_pulse(id_list.size(), 1);
    dyn_comm_.SyncWrite(bus_watchdog, id_list, bus_watchtime_pulse);
    ROS_INFO("All servo will be stopped");
}

// 回転数が消えることを考慮して，モータをリブートする．
bool DynamixelHandler::ClearHardwareError(uint8_t id){
    if ( series_[id] != SERIES_X ) return false; // Xシリーズ以外は対応していない
    if ( ReadHardwareError(id) == 0b00000000 ) return true; // エラーがない場合は何もしない

    const auto now_pos = ReadPresentPosition(id); // 失敗すると0が返って危ないので成功した場合だけリブート処理を行う
    const bool pos_success = !dyn_comm_.timeout_last_read() && !dyn_comm_.comm_error_last_read();
    const auto now_offset = ReadHomingOffset(id); // 失敗すると0が返って危ないので成功した場合だけリブート処理を行う
    const bool offset_success = !dyn_comm_.timeout_last_read() && !dyn_comm_.comm_error_last_read();
    if ( pos_success && offset_success ) {
        int now_rot = (now_pos-now_offset+M_PI) / (2*M_PI);
        if (now_pos < -M_PI) now_rot--;
        const double offset = now_offset+now_rot*(2*M_PI);
        /*リブート処理*/dyn_comm_.Reboot(id); //** RAMのデータが消えるが，この処理の後は電源喪失と同じ扱いなので，ここでは気にしない．
        // homing offsetが書き込めるまで待機する．
        while ( !WriteHomingOffset(id, offset) && ros::ok() ) rsleep(0.01);
    }
    // 結果を確認
    bool is_clear = (ReadHardwareError(id) == 0b00000000);
    if (is_clear) ROS_INFO ("ID [%d] is cleared error", id);
    else          ROS_ERROR("ID [%d] failed to clear error", id);
    return is_clear;
}

// モータの動作モードを変更する．連続で変更するときは1秒のインターバルを入れる
bool DynamixelHandler::ChangeOperatingMode(uint8_t id, DynamixelOperatingMode mode){
    if ( series_[id] != SERIES_X ) return false; // Xシリーズ以外は対応していない
    if ( op_mode_[id] == mode ) return true; // 既に同じモードの場合は何もしない
    if ( fabs((when_op_mode_updated_[id] - Time::now()).toSec()) < 1.0 ) rsleep(1.0); // 1秒以内に変更した場合は1秒待つ
    // 変更前のトルク状態を確認
    const bool is_enable = (ReadTorqueEnable(id) == TORQUE_ENABLE); // read失敗しても0が返ってくるので問題ない
    WriteTorqueEnable(id, false);
    /*モード変更*/WriteOperatingMode(id, mode);  //**RAMのデータが消えるので注意, これは電源喪失とは異なるのでRAMデータの回復を入れる
    // cmd_values_を全部書き込んで，本体とこのプログラムの同期行う．
    WriteGoalPWM(id, cmd_values_[id][GOAL_PWM]);
    WriteGoalCurrent(id, cmd_values_[id][GOAL_CURRENT]);
    WriteGoalVelocity(id, cmd_values_[id][GOAL_VELOCITY]);
    WriteProfileAcc(id, cmd_values_[id][PROFILE_ACC]);
    WriteProfileVel(id, cmd_values_[id][PROFILE_VEL]);
    WriteGoalPosition(id, cmd_values_[id][GOAL_POSITION]);
    // WriteGains(id, opt_gain_[id]);　// ** Gain値のデフォルトも変わる．面倒な．．．
    WriteTorqueEnable(id, is_enable);
    // 結果を確認
    bool is_changed = (ReadOperatingMode(id) == mode);
    if ( is_changed ) {
        op_mode_[id] = mode;
        when_op_mode_updated_[id] = Time::now();
        // if ( mode==OPERATING_MODE_CURRENT ) WriteBusWatchdog(id, 2500 /*ms*/);
        // if ( mode==OPERATING_MODE_VELOCITY) WriteBusWatchdog(id, 2500 /*ms*/);
        ROS_INFO("ID [%d] is changed operating mode [%d]", id, mode);
    } else {
        ROS_ERROR("ID [%d] failed to change operating mode", id); 
    }
    return is_changed;
}

// モータを停止させてからトルクを入れる．
bool DynamixelHandler::TorqueOn(uint8_t id){
    if ( series_[id] != SERIES_X ) return false; // Xシリーズ以外は対応していない
    // dynamixel内のgoal値とこのプログラム内のcmd_values_を一致させる．
    const auto now_pos = ReadPresentPosition(id); // 失敗すると0が返って危ないので確認する
    if ( !( dyn_comm_.timeout_last_read() || dyn_comm_.comm_error_last_read() )){
        // 急に動き出さないように，以下のcmd_values_を設定する
        cmd_values_[id][GOAL_POSITION] = now_pos; // トルクがオフならDynamixel本体のgoal_positionはpresent_positionと一致している．
        if (op_mode_[id]==OPERATING_MODE_VELOCITY) cmd_values_[id][GOAL_VELOCITY] = 0.0;
        if (op_mode_[id]==OPERATING_MODE_CURRENT ) cmd_values_[id][GOAL_CURRENT]  = 0.0;
        if (op_mode_[id]==OPERATING_MODE_PWM     ) cmd_values_[id][GOAL_PWM]      = 0.0;
        // cmd_values_を全部書き込んで，本体とこのプログラムの同期行う．
        WriteGoalPWM(id, cmd_values_[id][GOAL_PWM]);
        WriteGoalCurrent(id, cmd_values_[id][GOAL_CURRENT]);
        WriteGoalVelocity(id, cmd_values_[id][GOAL_VELOCITY]);
        WriteProfileAcc(id, cmd_values_[id][PROFILE_ACC]);
        WriteProfileVel(id, cmd_values_[id][PROFILE_VEL]);
        WriteGoalPosition(id, cmd_values_[id][GOAL_POSITION]);
        // WriteGains(id, opt_gain_[id]); 　// その他電源喪失時に消えるデータを念のため書き込む
        /*トルクを入れる*/WriteTorqueEnable(id, true);
    }
    // 結果を確認
    tq_mode_[id] = (ReadTorqueEnable(id) == TORQUE_ENABLE);
    if ( !tq_mode_[id] ) ROS_ERROR("ID [%d] failed to enable torque", id);
    return tq_mode_[id];
}

// トルクを切る
bool DynamixelHandler::TorqueOff(uint8_t id){
    if ( series_[id] != SERIES_X ) return false; // Xシリーズ以外は対応していない
    // トルクを切る
    WriteTorqueEnable(id, false);
    // 結果を確認
    tq_mode_[id] = (ReadTorqueEnable(id) == TORQUE_DISABLE);
    if ( !tq_mode_[id] ) ROS_ERROR("ID [%d] failed to disable torque", id);
    return tq_mode_[id];
}

//* 基本機能たち Read

uint8_t DynamixelHandler::ReadHardwareError(uint8_t id){
    return dyn_comm_.tryRead(hardware_error_status, id);
}

bool DynamixelHandler::ReadTorqueEnable(uint8_t id){
    return dyn_comm_.tryRead(torque_enable, id);
}

double DynamixelHandler::ReadPresentPWM(uint8_t id){
    const auto pwm_pulse = dyn_comm_.tryRead(present_pwm, id);
    return present_pwm.pulse2val(pwm_pulse, model_[id]);
}

double DynamixelHandler::ReadPresentCurrent(uint8_t id){
    const auto cur_pulse = dyn_comm_.tryRead(present_current, id);
    return present_current.pulse2val(cur_pulse, model_[id]);
}

double DynamixelHandler::ReadPresentVelocity(uint8_t id){
    const auto vel_pulse = dyn_comm_.tryRead(present_velocity, id);
    return present_velocity.pulse2val(vel_pulse, model_[id]);
}

double DynamixelHandler::ReadPresentPosition(uint8_t id){
    const auto pos_pulse = dyn_comm_.tryRead(present_position, id);
    return present_position.pulse2val(pos_pulse, model_[id]);
}

double DynamixelHandler::ReadGoalPWM(uint8_t id){
    const auto pwm_pulse = dyn_comm_.tryRead(goal_pwm, id);
    return goal_pwm.pulse2val(pwm_pulse, model_[id]);
}

double DynamixelHandler::ReadGoalCurrent(uint8_t id){
    const auto cur_pulse = dyn_comm_.tryRead(goal_current, id);
    return goal_current.pulse2val(cur_pulse, model_[id]);
}

double DynamixelHandler::ReadGoalVelocity(uint8_t id){
    const auto vel_pulse = dyn_comm_.tryRead(goal_velocity, id);
    return goal_velocity.pulse2val(vel_pulse, model_[id]);
}

double DynamixelHandler::ReadGoalPosition(uint8_t id){
    const auto pos_pulse = dyn_comm_.tryRead(goal_position, id);
    return goal_position.pulse2val(pos_pulse, model_[id]);
}

double DynamixelHandler::ReadProfileAcc(uint8_t id){
    const auto acc_pulse = dyn_comm_.tryRead(profile_acceleration, id);
    return profile_acceleration.pulse2val(acc_pulse, model_[id]);
}

double DynamixelHandler::ReadProfileVel(uint8_t id){
    const auto vel_pulse = dyn_comm_.tryRead(profile_velocity, id);
    return profile_velocity.pulse2val(vel_pulse, model_[id]);
}

double DynamixelHandler::ReadHomingOffset(uint8_t id){
    const auto offset_pulse = dyn_comm_.tryRead(homing_offset, id);
    return homing_offset.pulse2val(offset_pulse, model_[id]);
}

uint8_t DynamixelHandler::ReadOperatingMode(uint8_t id){
    return dyn_comm_.tryRead(operating_mode, id);
}

//* 基本機能たち Write

bool DynamixelHandler::WriteGoalPosition(uint8_t id, double pos){
    const auto pos_pulse = goal_position.val2pulse(pos, model_[id]);
    return dyn_comm_.tryWrite(goal_position, id, pos_pulse);
}

bool DynamixelHandler::WriteTorqueEnable(uint8_t id, bool enable){
    return dyn_comm_.tryWrite(torque_enable, id, enable ? TORQUE_ENABLE : TORQUE_DISABLE);
}

bool DynamixelHandler::WriteGoalPWM(uint8_t id, double pwm){
    const auto pwm_pulse = goal_pwm.val2pulse(pwm, model_[id]);
    return dyn_comm_.tryWrite(goal_pwm, id, pwm_pulse);
}

bool DynamixelHandler::WriteGoalCurrent(uint8_t id, double cur){
    const auto cur_pulse = goal_current.val2pulse(cur, model_[id]);
    return dyn_comm_.tryWrite(goal_current, id, cur_pulse);
}

bool DynamixelHandler::WriteGoalVelocity(uint8_t id, double vel){
    const auto vel_pulse = goal_velocity.val2pulse(vel, model_[id]);
    return dyn_comm_.tryWrite(goal_velocity, id, vel_pulse);
}

bool DynamixelHandler::WriteProfileAcc(uint8_t id, double acc){
    const auto acc_pulse = profile_acceleration.val2pulse(acc, model_[id]);
    return dyn_comm_.tryWrite(profile_acceleration, id, acc_pulse);
}

bool DynamixelHandler::WriteProfileVel(uint8_t id, double vel){
    const auto vel_pulse = profile_velocity.val2pulse(vel, model_[id]);
    return dyn_comm_.tryWrite(profile_velocity, id, vel_pulse);
}

bool DynamixelHandler::WriteHomingOffset(uint8_t id, double offset){
    const auto offset_pulse = homing_offset.val2pulse(offset, model_[id]);
    return dyn_comm_.tryWrite(homing_offset, id, offset_pulse);
}

bool DynamixelHandler::WriteOperatingMode(uint8_t id, uint8_t mode){ 
    return dyn_comm_.tryWrite(operating_mode, id, mode); 
}

bool DynamixelHandler::WriteBusWatchdog(uint8_t id, double time){
    const auto time_pulse = bus_watchdog.val2pulse(time, model_[id]);
    return dyn_comm_.tryWrite(bus_watchdog, id, time_pulse);
}

bool DynamixelHandler::WriteGains(uint8_t id, array<int64_t, 7> gains){
    bool is_success = true;
    is_success &= dyn_comm_.tryWrite(velocity_i_gain, id, gains[VELOCITY_I_GAIN]);
    is_success &= dyn_comm_.tryWrite(velocity_p_gain, id, gains[VELOCITY_P_GAIN]);
    is_success &= dyn_comm_.tryWrite(position_d_gain, id, gains[POSITION_D_GAIN]);
    is_success &= dyn_comm_.tryWrite(position_i_gain, id, gains[POSITION_I_GAIN]);
    is_success &= dyn_comm_.tryWrite(position_p_gain, id, gains[POSITION_P_GAIN]);
    is_success &= dyn_comm_.tryWrite(feedforward_acc_gain, id, gains[FEEDFORWARD_ACC_GAIN]);
    is_success &= dyn_comm_.tryWrite(feedforward_vel_gain, id, gains[FEEDFORWARD_VEL_GAIN]);
    return is_success;
}
