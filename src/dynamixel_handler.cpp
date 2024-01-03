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
    ROS_INFO("All servo will be stopped,");
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
        dyn_comm_.Reboot(id); //** RAMのデータが消えるが，この処理の後は電源喪失と同じ扱いなので，ここでは気にしない．
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
    const bool before = ReadTorqueEnable(id); // read失敗しても0が返ってくるので問題ない
    WriteTorqueEnable(id, TORQUE_DISABLE);
    /*モード変更*/WriteOperatingMode(id, mode);  //**RAMのデータが消えるので注意, これは電源喪失とは異なるのでRAMデータの回復を入れる
    // cmd_values_を全部書き込んで，本体とこのプログラムの同期行う．
    WriteGoalPWM(id, cmd_values_[id][GOAL_PWM]);
    WriteGoalCurrent(id, cmd_values_[id][GOAL_CURRENT]);
    WriteGoalVelocity(id, cmd_values_[id][GOAL_VELOCITY]);
    WriteProfileAcc(id, cmd_values_[id][PROFILE_ACC]);
    WriteProfileVel(id, cmd_values_[id][PROFILE_VEL]);
    WriteGoalPosition(id, cmd_values_[id][GOAL_POSITION]);
    // WriteGains(id, opt_gain_[id]);　// ** Gain値のデフォルトも変わる．面倒な．．．
    WriteTorqueEnable(id, before);
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
        /*トルクを入れる*/WriteTorqueEnable(id, TORQUE_ENABLE);
    }
    // 結果を確認
    bool is_enable = (ReadTorqueEnable(id) == TORQUE_ENABLE);
    if ( !is_enable ) ROS_ERROR("ID [%d] failed to enable torque", id);
    return is_enable;
}

// トルクを切る
bool DynamixelHandler::TorqueOff(uint8_t id){
    if ( series_[id] != SERIES_X ) return false; // Xシリーズ以外は対応していない
    // トルクを切る
    WriteTorqueEnable(id, TORQUE_DISABLE);
    // 結果を確認
    bool is_disable = (ReadTorqueEnable(id) == TORQUE_DISABLE);
    if ( !is_disable ) ROS_ERROR("ID [%d] failed to disable torque", id);
    return is_disable;
}

//* 基本機能たち

uint8_t DynamixelHandler::ReadHardwareError(uint8_t id){
    return dyn_comm_.tryRead(hardware_error_status, id);
}

double DynamixelHandler::ReadPresentPosition(uint8_t id){
    auto pos_pulse = dyn_comm_.tryRead(present_position, id);
    return present_position.pulse2val(pos_pulse, model_[id]);
}

bool DynamixelHandler::WriteGoalPosition(uint8_t id, double pos){
    auto pos_pulse = goal_position.val2pulse(pos, model_[id]);
    return dyn_comm_.tryWrite(goal_position, id, pos_pulse);
}

double DynamixelHandler::ReadHomingOffset(uint8_t id){
    auto offset_pulse = dyn_comm_.tryRead(homing_offset, id);
    return homing_offset.pulse2val(offset_pulse, model_[id]);
}

bool DynamixelHandler::WriteHomingOffset(uint8_t id, double offset){
    auto offset_pulse = homing_offset.val2pulse(offset, model_[id]);
    return dyn_comm_.tryWrite(homing_offset, id, offset_pulse);
}

double DynamixelHandler::ReadPresentVelocity(uint8_t id){
    auto vel_pulse = dyn_comm_.tryRead(present_velocity, id);
    return present_velocity.pulse2val(vel_pulse, model_[id]);
}

bool DynamixelHandler::WriteGoalVelocity(uint8_t id, double vel){
    auto vel_pulse = goal_velocity.val2pulse(vel, model_[id]);
    return dyn_comm_.tryWrite(goal_velocity, id, vel_pulse);
}

double DynamixelHandler::ReadPresentCurrent(uint8_t id){
    auto cur_pulse = dyn_comm_.tryRead(present_current, id);
    return present_current.pulse2val(cur_pulse, model_[id]);
}

bool DynamixelHandler::WriteGoalCurrent(uint8_t id, double cur){
    auto cur_pulse = goal_current.val2pulse(cur, model_[id]);
    return dyn_comm_.tryWrite(goal_current, id, cur_pulse);
}

bool DynamixelHandler::ReadTorqueEnable(uint8_t id){
    return dyn_comm_.tryRead(torque_enable, id);
}

bool DynamixelHandler::WriteTorqueEnable(uint8_t id, bool enable){
    return dyn_comm_.tryWrite(torque_enable, id, enable ? TORQUE_ENABLE : TORQUE_DISABLE);
}

uint8_t DynamixelHandler::ReadOperatingMode(uint8_t id){
    return dyn_comm_.tryRead(operating_mode, id);
}

bool DynamixelHandler::WriteOperatingMode(uint8_t id, uint8_t mode){ 
    return dyn_comm_.tryWrite(operating_mode, id, mode); 
}

bool DynamixelHandler::WriteGoalPWM(uint8_t id, double pwm){
    auto pwm_pulse = goal_pwm.val2pulse(pwm, model_[id]);
    return dyn_comm_.tryWrite(goal_pwm, id, pwm_pulse);
}

bool DynamixelHandler::WriteProfileVel(uint8_t id, double vel){
    auto vel_pulse = profile_velocity.val2pulse(vel, model_[id]);
    return dyn_comm_.tryWrite(profile_velocity, id, vel_pulse);
}

bool DynamixelHandler::WriteProfileAcc(uint8_t id, double acc){
    auto acc_pulse = profile_acceleration.val2pulse(acc, model_[id]);
    return dyn_comm_.tryWrite(profile_acceleration, id, acc_pulse);
}

bool DynamixelHandler::WriteBusWatchdog(uint8_t id, double time){
    auto time_pulse = bus_watchdog.val2pulse(time, model_[id]);
    return dyn_comm_.tryWrite(bus_watchdog, id, time_pulse);
}

// bool DynamixelHandler::WriteGains(uint8_t id, array<int64_t, 7> gains){
    // if ( gains.size() != 8 ) return false;
    // bool is_success = true;
    // is_success &= dyn_comm_.tryWrite(velocity_i_gain, id, gains[0]);
    // is_success &= dyn_comm_.tryWrite(velocity_p_gain, id, gains[1]);
    // is_success &= dyn_comm_.tryWrite(position_d_gain, id, gains[2]);
    // is_success &= dyn_comm_.tryWrite(position_i_gain, id, gains[3]);
    // is_success &= dyn_comm_.tryWrite(position_p_gain, id, gains[4]);
    // is_success &= dyn_comm_.tryWrite(feedforward_acc_gain, id, gains[5]);
    // is_success &= dyn_comm_.tryWrite(feedforward_vel_gain, id, gains[6]);
    // return is_success;
// }

//* Main loop 内で使う全モータへの一括読み書き関数たち

/**
 * @func SyncWriteCommandValues
 * @brief 指定した範囲のコマンド値を書き込む
 * @param list_write_cmd 書き込むコマンドのEnumのリスト
*/
void DynamixelHandler::SyncWriteCommandValues(set<CmdValueIndex>& list_write_cmd){
    // 空なら即時return
    if ( list_write_cmd.empty() ) return;
    // 書き込む範囲のイテレータを取得, 分割書き込みが有効な場合書き込む範囲を1つ目のみに制限,残りは再帰的に処理する．
    auto start = min_element(list_write_cmd.begin(), list_write_cmd.end());   
    auto end = ( use_split_write_ ) ? start 
               : max_element(list_write_cmd.begin(), list_write_cmd.end());
    // 書き込みに必要な変数を用意
    vector<DynamixelAddress> target_cmd_dp_list;  // 書き込むコマンドのアドレスのベクタ
    map<uint8_t, vector<int64_t>> id_cmd_vec_map; // id と 書き込むデータのベクタのマップ
    for (size_t cmd = *start; cmd <= *end; cmd++) { // アドレスのベクタと，データのベクタの並びは対応している必要があるので，同一のループで作成する．
        const auto dp = cmd_dp_list[cmd];
        target_cmd_dp_list.push_back(dp); 
        for (auto id : id_list_) if ( series_[id]==SERIES_X ) {
            if ( !is_cmd_updated_[id] ) continue; // 更新されていない場合はスキップ
            const auto pulse  = dp.val2pulse( cmd_values_[id][cmd], model_[id]);
            id_cmd_vec_map[id].push_back( pulse );
        }
    }
    //id_cmd_vec_mapの中身を確認
    if ( varbose_write_cmd_ ) {
        char header[100]; sprintf(header, "[%d] servo(s) will be written", (int)id_cmd_vec_map.size());
        auto ss = control_table_layout(width_log_, id_cmd_vec_map, target_cmd_dp_list, string(header));
        ROS_INFO_STREAM(ss);
    }
    // SyncWriteでまとめて書き込み
    dyn_comm_.SyncWrite(target_cmd_dp_list, id_cmd_vec_map);
    // 今回書き込みした範囲を消去して残りを再帰的に処理, use_split_write_=falseの場合は全て削除されるので,再帰しない
    list_write_cmd.erase(start, ++end);
    SyncWriteCommandValues(list_write_cmd);
    // 後処理，再帰の終端で実行される
    is_cmd_updated_.clear();
}

void DynamixelHandler::SyncWriteOption_Mode(){
    return;
}

void DynamixelHandler::SyncWriteOption_Gain(){
    return;
}

void DynamixelHandler::SyncWriteOption_Limit(){
    return;
}
using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

/**
 * @func SyncReadStateValues
 * @brief 指定した範囲の状態値を読み込む
 * @param list_read_state 読み込む状態値のEnumのリスト
 * @return 読み取りの成功率, なんで再帰で頑張って実装してるんだろう．．．
*/
double DynamixelHandler::SyncReadStateValues(set<StValueIndex> list_read_state){
    // 空なら即時return
    if ( list_read_state.empty() ) return 1.0;
    // 読み込む範囲のstate_dp_listのインデックスを取得
    auto start = min_element(list_read_state.begin(), list_read_state.end());
    auto end = ( use_split_read_ ) ? start
               : max_element(list_read_state.begin(), list_read_state.end());
    // 読み込みに必要な変数を用意
    vector<DynamixelAddress> target_state_dp_list;
    for (size_t st=*start; st<=*end; st++) target_state_dp_list.push_back(state_dp_list[st]);
    vector<uint8_t> target_id_list;
    for (auto id : id_list_) if ( series_[id]==SERIES_X ) target_id_list.push_back(id);
    // SyncReadでまとめて読み込み
    const auto id_st_vec_map = ( use_fast_read_ ) // fast read を使うかどうか．　途中で切り替えるとtimeout後に来るデータによってSyncReadが何度も失敗するので注意
        ? dyn_comm_.SyncRead_fast(target_state_dp_list, target_id_list)
        : dyn_comm_.SyncRead     (target_state_dp_list, target_id_list);
    const size_t N_total = target_id_list.size();
    const size_t N_suc   = id_st_vec_map.size();
    const bool is_timeout_  = dyn_comm_.timeout_last_read();
    const bool is_comm_err_ = dyn_comm_.comm_error_last_read();
    has_hardware_err_ = dyn_comm_.hardware_error_last_read();
    // 通信エラーの表示
    if ( varbose_read_st_err_ ) if ( is_timeout_ || is_comm_err_ ) {
        vector<uint8_t> failed_id_list;
        for ( auto id : target_id_list ) if ( id_st_vec_map.find(id) == id_st_vec_map.end() ) failed_id_list.push_back(id);
        char header[99]; sprintf(header, "[%d] servo(s) failed to read", (int)(N_total - N_suc));
        auto ss = id_list_layout(failed_id_list, string(header)+( is_timeout_ ? " (time out)" : " (some kind packet error)"));
        ROS_WARN_STREAM(ss);
    }
    // id_st_vec_mapの中身を確認
    if ( varbose_read_st_ ) if ( N_suc>0 ) {
        char header[99]; sprintf(header, "[%d] servo(s) are read", (int)N_suc);
        auto ss = control_table_layout(width_log_, id_st_vec_map, target_state_dp_list, string(header));
        ROS_INFO_STREAM(ss);
        if ( has_hardware_err_ ) ROS_WARN("Hardware Error are detected");
    }
    // state_values_に反映
    const int num_state = *end-*start+1;
    for (int i = 0; i < num_state; i++) {
        const auto dp = target_state_dp_list[i];
        for (auto pair : id_st_vec_map) {
            const uint8_t id = pair.first;
            const int64_t data_int = pair.second[i];
            state_values_[id][*start+i] = dp.pulse2val( data_int, model_[id]);
        }
    }
    // 今回読み込んだ範囲を消去して残りを再帰的に処理, use_split_read_=falseの場合は全て削除されるので,再帰しない
    list_read_state.erase(start, ++end);
    double suc_rate_prev = SyncReadStateValues(list_read_state)*list_read_state.size() / (num_state+list_read_state.size());
    return suc_rate_prev + N_suc/(double)N_total*num_state/(num_state+list_read_state.size());
}

/**
 * @func SyncReadHardwareError
 * @brief ハードウェアエラーを読み込む
 * @return 読み取りの成功率
*/
double DynamixelHandler::SyncReadHardwareErrors(){
    if ( !has_hardware_err_ ) { hardware_error_.clear(); return 1.0; } // 事前にエラーが検出できていない場合は省略

    vector<uint8_t> target_id_list;
    for (int id : id_list_) if ( series_[id]==SERIES_X ) target_id_list.push_back(id);
    
    auto id_error_map =  ( use_fast_read_ ) 
        ? dyn_comm_.SyncRead_fast(hardware_error_status, target_id_list)
        : dyn_comm_.SyncRead     (hardware_error_status, target_id_list);

    if ( dyn_comm_.timeout_last_read() ) return 0.0; // 読み込み失敗

    //  hardware_error_に反映
    for ( auto pair : id_error_map ){
        const uint8_t id = pair.first;
        const uint8_t error = pair.second;
        if ((error >> HARDWARE_ERROR_INPUT_VOLTAGE)     & 0b1 ) hardware_error_[id][INPUT_VOLTAGE     ] = true;
        if ((error >> HARDWARE_ERROR_MOTOR_HALL_SENSOR) & 0b1 ) hardware_error_[id][MOTOR_HALL_SENSOR ] = true;
        if ((error >> HARDWARE_ERROR_OVERHEATING)       & 0b1 ) hardware_error_[id][OVERHEATING       ] = true;
        if ((error >> HARDWARE_ERROR_MOTOR_ENCODER)     & 0b1 ) hardware_error_[id][MOTOR_ENCODER     ] = true;
        if ((error >> HARDWARE_ERROR_ELECTRONICAL_SHOCK)& 0b1 ) hardware_error_[id][ELECTRONICAL_SHOCK] = true;
        if ((error >> HARDWARE_ERROR_OVERLOAD)          & 0b1 ) hardware_error_[id][OVERLOAD          ] = true;
    }

    // コンソールへの表示
    if ( varbose_read_hwerr_ ) {
        ROS_WARN("Hardware error are Checked");
        for (auto id : target_id_list) {
            if (hardware_error_[id][INPUT_VOLTAGE     ]) ROS_ERROR(" * servo id [%d] has INPUT_VOLTAGE error",      id);
            if (hardware_error_[id][MOTOR_HALL_SENSOR ]) ROS_ERROR(" * servo id [%d] has MOTOR_HALL_SENSOR error",  id);
            if (hardware_error_[id][OVERHEATING       ]) ROS_ERROR(" * servo id [%d] has OVERHEATING error",        id);
            if (hardware_error_[id][MOTOR_ENCODER     ]) ROS_ERROR(" * servo id [%d] has MOTOR_ENCODER error",      id);
            if (hardware_error_[id][ELECTRONICAL_SHOCK]) ROS_ERROR(" * servo id [%d] has ELECTRONICAL_SHOCK error", id);
            if (hardware_error_[id][OVERLOAD          ]) ROS_ERROR(" * servo id [%d] has OVERLOAD error",           id);
        }
    }
    return id_error_map.size()/double(target_id_list.size());
}

double DynamixelHandler::SyncReadOption_Mode(){
    return 1.0;
}

double DynamixelHandler::SyncReadOption_Gain(){
    return 1.0;
}

double DynamixelHandler::SyncReadOption_Limit(){
    vector<uint8_t> target_id_list;
    for (int id : id_list_) if ( series_[id]==SERIES_X ) target_id_list.push_back(id);

    auto id_limit_vec_map = ( use_fast_read_ )
        ? dyn_comm_.SyncRead_fast(opt_limit_dp_list, target_id_list)
        : dyn_comm_.SyncRead     (opt_limit_dp_list, target_id_list);
    const bool is_timeout   = dyn_comm_.timeout_last_read();
    const bool has_comm_err = dyn_comm_.comm_error_last_read();
    // 通信エラーの表示
    if ( varbose_read_opt_err_ ) if ( has_comm_err || is_timeout ) {
        vector<uint8_t> failed_id_list;
        for ( auto id : target_id_list ) if ( id_limit_vec_map.find(id) == id_limit_vec_map.end() ) failed_id_list.push_back(id);
        char header[100]; sprintf(header, "[%d] servo(s) failed to read", (int)(target_id_list.size() - id_limit_vec_map.size()));
        auto ss = id_list_layout(failed_id_list, string(header) + (is_timeout? " (time out)" : " (some kind packet error)"));
        ROS_WARN_STREAM(ss);
    }
    // ACCELERATION_LIMITに関してだけ修正を入れる．0はほぼあり得ないかつ0の時profile_accの設定ができないので，適当に大きな値に変更する．
    vector<uint8_t> fixed_id_list;
    for (auto pair : id_limit_vec_map) {
        const uint8_t id = pair.first;
        const int64_t acc = pair.second[ACCELERATION_LIMIT];
        if ( acc != 0 ) continue;
        fixed_id_list.push_back(id);
        id_limit_vec_map[id][ACCELERATION_LIMIT] = 32767;
    }
    // id_limit_vec_mapの中身を確認
    if ( varbose_read_opt_ ) if ( id_limit_vec_map.size()>0 ) {
        char header[100]; sprintf(header, "[%d] servo(s) are read", (int)id_limit_vec_map.size());
        auto ss = control_table_layout(width_log_, id_limit_vec_map, opt_limit_dp_list, string(header));
        ROS_INFO_STREAM(ss);
        if ( !fixed_id_list.empty() ) {
            char header[100]; sprintf(header,"\n[%d] servo(s)' accelerarion_limit is 0, change to 32767", (int)fixed_id_list.size());
            auto ss = id_list_layout(fixed_id_list, string(header));
            ROS_WARN_STREAM(ss);
        }
    }
    // option_limit_に反映
    for ( auto opt_lim=0; opt_lim<opt_limit_dp_list.size(); opt_lim++) {
        DynamixelAddress dp = opt_limit_dp_list[opt_lim];
        for (auto pair : id_limit_vec_map) {
            const uint8_t id = pair.first;
            const int64_t data_int = pair.second[opt_lim];
            option_limit_[id][opt_lim] = dp.pulse2val( data_int, model_[id] );
        }
    }
    return id_limit_vec_map.size() / (double)target_id_list.size();
}

//* ROS関係

void DynamixelHandler::CallBackDxlCommand(const dynamixel_handler::DynamixelCommand& msg) {
    vector<uint8_t> id_list;
    if ( msg.id_list.empty() || msg.id_list[0]==0xFE) for (auto id : id_list_) id_list.push_back(id);
                                                 else for (auto id : msg.id_list) id_list.push_back(id);
    char header[100]; sprintf(header, "Command [%s] \n (id_list=[] or [254] means all IDs)", msg.command.c_str());
    ROS_INFO_STREAM(id_list_layout(id_list, string(header)));
    if (msg.command == "clear_error" || msg.command == "CE")
        for (auto id : id_list) { ClearHardwareError(id); TorqueOn(id);}
    if (msg.command == "torque_on"   || msg.command == "TON") 
        for (auto id : id_list) TorqueOn(id);
    if (msg.command == "torque_off"  || msg.command == "TOFF")
        for (auto id : id_list) TorqueOff(id);
    if (msg.command == "enable") 
        for (auto id : id_list) WriteTorqueEnable(id, TORQUE_ENABLE);
    if (msg.command == "disable")
        for (auto id : id_list) WriteTorqueEnable(id, TORQUE_DISABLE);
    if (msg.command == "reboot") 
        for (auto id : id_list) dyn_comm_.Reboot(id);
}

void DynamixelHandler::CallBackDxlCommand_Profile(const dynamixel_handler::DynamixelCommand_Profile& msg) {
    if (varbose_callback_) ROS_INFO("CallBackDxlCommand_Profile"); // msg.id_listと同じサイズの奴だけ処理する
    const bool do_process_vel = msg.id_list.size() == msg.profile_velocity__deg_s.size();
    const bool do_process_acc = msg.id_list.size() == msg.profile_acceleration__deg_ss.size();
    if ( !do_process_vel && !do_process_acc ) { if (varbose_callback_) ROS_ERROR(" - Element size dismatch"); return;}

    for (int i=0; i<msg.id_list.size(); i++){
        int id = msg.id_list[i];
        auto limit = option_limit_[id];
        if ( do_process_vel ){
            auto vel = msg.profile_velocity__deg_s[i];
            cmd_values_[id][PROFILE_VEL] = clamp( deg2rad(vel), -limit[VELOCITY_LIMIT], limit[VELOCITY_LIMIT] );
            is_cmd_updated_[id] = true;
            list_write_cmd_.insert(PROFILE_VEL);
        }
        if ( do_process_acc ){
            auto acc = msg.profile_acceleration__deg_ss[i];
            cmd_values_[id][PROFILE_ACC] = clamp( deg2rad(acc), -limit[ACCELERATION_LIMIT], limit[ACCELERATION_LIMIT] );
            is_cmd_updated_[id] = true;
            list_write_cmd_.insert(PROFILE_ACC);
        }
    }
    if (varbose_callback_) ROS_INFO(" - %d servo(s) profile are updated", (int)msg.id_list.size());
}

void DynamixelHandler::CallBackDxlCommand_X_Position(const dynamixel_handler::DynamixelCommand_X_ControlPosition& msg) {
    if (varbose_callback_) ROS_INFO("msg generate time: %f", msg.stamp.toSec());  // ↓msg.id_listと同じサイズの奴だけ処理する
    if ( msg.id_list.size() != msg.position__deg.size() ) { if (varbose_callback_) ROS_ERROR(" - Element size dismatch"); return;}

    for (int i = 0; i < msg.id_list.size(); i++) {
        int id = msg.id_list[i];
        auto pos = msg.position__deg[i];
        auto limit = option_limit_[id];
        cmd_values_[id][GOAL_POSITION] = clamp(deg2rad(pos), limit[MIN_POSITION_LIMIT], limit[MAX_POSITION_LIMIT]);
        is_cmd_updated_[id] = true;
        list_write_cmd_.insert(GOAL_POSITION);
        ChangeOperatingMode(id, OPERATING_MODE_POSITION); // 副作用で変更する場合だけトルクが入ってしまう．
    }
    if (varbose_callback_) ROS_INFO(" - %d servo(s) goal_position are updated", (int)msg.id_list.size());
}

void DynamixelHandler::CallBackDxlCommand_X_Velocity(const dynamixel_handler::DynamixelCommand_X_ControlVelocity& msg) {
    if (varbose_callback_) ROS_INFO("CallBackDxlCommand_X_Velocity"); // msg.id_listと同じサイズの奴だけ処理する
    if ( msg.id_list.size() != msg.velocity__deg_s.size() ) { if (varbose_callback_) ROS_ERROR(" - Element size dismatch"); return;}

    for (int i = 0; i < msg.id_list.size(); i++) {
        int id = msg.id_list[i];
        auto vel = msg.velocity__deg_s[i];
        auto limit = option_limit_[id];
        cmd_values_[id][GOAL_VELOCITY] = clamp(deg2rad(vel), -limit[VELOCITY_LIMIT], limit[VELOCITY_LIMIT]);
        is_cmd_updated_[id] = true;
        list_write_cmd_.insert(GOAL_VELOCITY);
        ChangeOperatingMode(id, OPERATING_MODE_VELOCITY);
    }
    if (varbose_callback_) ROS_INFO(" - %d servo(s) goal_velocity are updated", (int)msg.id_list.size());
}

void DynamixelHandler::CallBackDxlCommand_X_Current(const dynamixel_handler::DynamixelCommand_X_ControlCurrent& msg) {
    if (varbose_callback_) ROS_INFO("CallBackDxlCommand_X_Current"); // msg.id_listと同じサイズの奴だけ処理する
    if ( msg.id_list.size() != msg.current__mA.size() ) { if (varbose_callback_) ROS_ERROR(" - Element size dismatch"); return;}

    for (int i = 0; i < msg.id_list.size(); i++) {
        int id = msg.id_list[i];
        auto cur = msg.current__mA[i];
        auto limit = option_limit_[id];
        cmd_values_[id][GOAL_CURRENT] = clamp(cur, -limit[CURRENT_LIMIT], limit[CURRENT_LIMIT]);
        is_cmd_updated_[id] = true;
        list_write_cmd_.insert(GOAL_CURRENT);
        ChangeOperatingMode(id, OPERATING_MODE_CURRENT);
    }
    if (varbose_callback_) ROS_INFO(" - %d servo(s) goal_current are updated", (int)msg.id_list.size());
}

void DynamixelHandler::CallBackDxlCommand_X_CurrentPosition(const dynamixel_handler::DynamixelCommand_X_ControlCurrentPosition& msg) {
    if (varbose_callback_) ROS_INFO("CallBackDxlCommand_X_CurrentPosition"); // msg.id_listと同じサイズの奴だけ処理する
    const bool do_process_cur = msg.id_list.size() == msg.current__mA.size();
    const bool do_process_pos = msg.id_list.size() == msg.position__deg.size() || msg.id_list.size() == msg.rotation.size();
    if ( !do_process_cur && !do_process_pos ) { if (varbose_callback_) ROS_ERROR(" - Element size dismatch"); return;}

    for (int i = 0; i < msg.id_list.size(); i++) {
        int id = msg.id_list[i];
        auto limit = option_limit_[id];
        if ( do_process_pos ){
            auto pos = msg.id_list.size() == msg.position__deg.size() ? msg.position__deg[i] : 0.0;
            auto rot = msg.id_list.size() == msg.rotation.size()      ? msg.rotation[i]      : 0;
            is_cmd_updated_[id] = true;
            cmd_values_[id][GOAL_POSITION] = clamp( deg2rad(pos + rot * 360.0), -256*2*M_PI, 256*2*M_PI );
            list_write_cmd_.insert(GOAL_POSITION);
        }
        if ( do_process_cur ){
            auto cur = msg.current__mA[i];
            is_cmd_updated_[id] = true;
            cmd_values_[id][GOAL_CURRENT] = clamp(cur, 0.0, limit[CURRENT_LIMIT]);
            list_write_cmd_.insert(GOAL_CURRENT);
        }
        ChangeOperatingMode(id, OPERATING_MODE_CURRENT_BASE_POSITION);
    }
    if (varbose_callback_ && do_process_cur) ROS_INFO(" - %d servo(s) goal_current are updated", (int)msg.id_list.size());
    if (varbose_callback_ && do_process_pos) ROS_INFO(" - %d servo(s) goal_position are updated", (int)msg.id_list.size());
}

void DynamixelHandler::CallBackDxlCommand_X_ExtendedPosition(const dynamixel_handler::DynamixelCommand_X_ControlExtendedPosition& msg) {
    if (varbose_callback_) ROS_INFO("CallBackDxlCommand_X_ExtendedPosition");
    const bool do_process = msg.id_list.size() == msg.position__deg.size() || msg.id_list.size() == msg.rotation.size();
    if ( !do_process ) { if (varbose_callback_) ROS_ERROR(" - Element size dismatch"); return;}

    for (int i = 0; i < msg.id_list.size(); i++) {
        int id = msg.id_list[i];
        auto pos = msg.id_list.size() == msg.position__deg.size() ? msg.position__deg[i] : 0.0;
        auto rot = msg.id_list.size() == msg.rotation.size()      ? msg.rotation[i]      : 0;
        is_cmd_updated_[id] = true;
        cmd_values_[id][GOAL_POSITION] = clamp( deg2rad(pos + rot * 360.0), -256*2*M_PI, 256*2*M_PI );
        list_write_cmd_.insert(GOAL_POSITION);
        ChangeOperatingMode(id, OPERATING_MODE_EXTENDED_POSITION);
    }
    if (varbose_callback_) ROS_INFO(" - %d servo(s) goal_position are updated", (int)msg.id_list.size());
}

void DynamixelHandler::CallBackDxlOption_Gain(const dynamixel_handler::DynamixelOption_Gain& msg) {
    // if (varbose_callback_) ROS_INFO("CallBackDxlOption_Gain");
    bool is_any = false;
    if (msg.id_list.size() == msg.velocity_i_gain__pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.velocity_p_gain__pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.position_d_gain__pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.position_i_gain__pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.position_p_gain__pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.feedforward_acc_gain__pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.feedforward_vel_gain__pulse.size()){ is_any=true;}
    if (varbose_callback_) {
        //  if (is_any) ROS_INFO(" - %d servo(s) gain are updated", (int)msg.id_list.size());
        //  else                  ROS_ERROR(" - Element size dismatch");
    }
}

void DynamixelHandler::CallBackDxlOption_Limit(const dynamixel_handler::DynamixelOption_Limit& msg) {
    // if (varbose_callback_) ROS_INFO("CallBackDxlOption_Limit");
    bool is_any = false;

    if (msg.id_list.size() == msg.temperature_limit__degC.size()   ){is_any=true;}
    if (msg.id_list.size() == msg.max_voltage_limit__V.size()      ){is_any=true;}
    if (msg.id_list.size() == msg.min_voltage_limit__V.size()      ){is_any=true;}
    if (msg.id_list.size() == msg.pwm_limit__percent.size()        ){is_any=true;}
    if (msg.id_list.size() == msg.current_limit__mA.size()         ){is_any=true;}
    if (msg.id_list.size() == msg.acceleration_limit__deg_ss.size()){is_any=true;}
    if (msg.id_list.size() == msg.velocity_limit__deg_s.size()     ){is_any=true;}
    if (msg.id_list.size() == msg.max_position_limit__deg.size()   ){is_any=true;}
    if (msg.id_list.size() == msg.min_position_limit__deg.size()   ){is_any=true;}

    if (varbose_callback_) {
        //  if (is_any) ROS_INFO(" - %d servo(s) limit are updated", (int)msg.id_list.size());
        //  else                  ROS_ERROR(" - Element size dismatch");
    }
}

void DynamixelHandler::CallBackDxlOption_Mode(const dynamixel_handler::DynamixelOption_Mode& msg) {
 
}

void DynamixelHandler::BroadcastDxlState(){
    dynamixel_handler::DynamixelState msg;
    msg.stamp = ros::Time::now();
    for (auto id : id_list_) {
        msg.id_list.push_back(id);
        for (auto state : list_read_state_) switch(state) {
            case PRESENT_PWM:          msg.pwm__percent.push_back         (state_values_[id][PRESENT_PWM          ]    ); break;
            case PRESENT_CURRENT:      msg.current__mA.push_back          (state_values_[id][PRESENT_CURRENT      ]    ); break;
            case PRESENT_VELOCITY:     msg.velocity__deg_s.push_back      (state_values_[id][PRESENT_VELOCITY     ]/DEG); break;
            case PRESENT_POSITION:     msg.position__deg.push_back        (state_values_[id][PRESENT_POSITION     ]/DEG); break;
            case VELOCITY_TRAJECTORY:  msg.vel_trajectory__deg_s.push_back(state_values_[id][VELOCITY_TRAJECTORY  ]/DEG); break;
            case POSITION_TRAJECTORY:  msg.pos_trajectory__deg.push_back  (state_values_[id][POSITION_TRAJECTORY  ]/DEG); break;
            case PRESENT_TEMPERTURE:   msg.temperature__degC.push_back    (state_values_[id][PRESENT_TEMPERTURE   ]    ); break;
            case PRESENT_INPUT_VOLTAGE:msg.input_voltage__V.push_back     (state_values_[id][PRESENT_INPUT_VOLTAGE]    ); break;
        }
    }
    pub_state_.publish(msg);
}

void DynamixelHandler::BroadcastDxlError(){
    dynamixel_handler::DynamixelError msg;
    msg.stamp = ros::Time::now();
    for (auto id : id_list_) {
        if (hardware_error_[id][INPUT_VOLTAGE     ]) msg.input_voltage.push_back     (id);
        if (hardware_error_[id][MOTOR_HALL_SENSOR ]) msg.motor_hall_sensor.push_back (id);
        if (hardware_error_[id][OVERHEATING       ]) msg.overheating.push_back       (id);
        if (hardware_error_[id][MOTOR_ENCODER     ]) msg.motor_encoder.push_back     (id);
        if (hardware_error_[id][ELECTRONICAL_SHOCK]) msg.electronical_shock.push_back(id);
        if (hardware_error_[id][OVERLOAD          ]) msg.overload.push_back          (id);
    }
    pub_error_.publish(msg);
}

void DynamixelHandler::BroadcastDxlOption_Limit(){

}
void DynamixelHandler::BroadcastDxlOption_Gain(){

}
void DynamixelHandler::BroadcastDxlOption_Mode(){

}
