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

// 回転数が消えることを考慮して，モータをリブートする．
bool DynamixelHandler::ClearHardwareError(uint8_t id, DynamixelTorquePermission after){
    if ( ReadHardwareError(id) == 0b00000000 ) return true; // エラーがない場合は何もしない
    if ( series_[id] != SERIES_X ) return false; // Xシリーズ以外は対応していない

    auto now_pos = ReadPresentPosition(id);
    int now_rot = (now_pos+M_PI) / (2*M_PI);
    if (now_pos < -M_PI) now_rot--;

    dyn_comm_.Reboot(id);
    sleep_for(0.5s);

    WriteHomingOffset(id, now_rot*(2*M_PI));
    if( after == TORQUE_ENABLE ) TorqueOn(id);

    bool is_clear = ReadHardwareError(id) == 0b00000000;
    if (is_clear) ROS_INFO ("ID [%d] is cleared error", id);
    else          ROS_ERROR("ID [%d] failed to clear error", id);
    return is_clear;
}

// モータを停止させてからトルクを入れる．
bool DynamixelHandler::TorqueOn(uint8_t id){
    if ( series_[id] != SERIES_X ) return false; // Xシリーズ以外は対応していない
    // 角度の同期と速度の停止
    if ( !StopRotation(id) ) return false;//この処理が失敗すると危険なので，トルク入れない．
    // トルクを入れる
    WriteTorqueEnable(id, TORQUE_ENABLE);
    // 結果を確認
    bool is_enable = TORQUE_ENABLE == ReadTorqueEnable(id);
    if ( !is_enable ) ROS_ERROR("ID [%d] failed to enable torque", id);
    return is_enable;
}

// トルクを切る
bool DynamixelHandler::TorqueOff(uint8_t id){
    if ( series_[id] != SERIES_X ) return false; // Xシリーズ以外は対応していない
    // トルクを切る
    WriteTorqueEnable(id, TORQUE_DISABLE);
    // 結果を確認
    bool is_disable = TORQUE_DISABLE == ReadTorqueEnable(id);
    if ( !is_disable ) ROS_ERROR("ID [%d] failed to disable torque", id);
    return is_disable;
}

// モータの動作モードを変更する．連続で変更するときは1秒のインターバルを入れる
bool DynamixelHandler::ChangeOperatingMode(uint8_t id, DynamixelOperatingMode mode, DynamixelTorquePermission after){
    if ( series_[id] != SERIES_X ) return false; // Xシリーズ以外は対応していない
    if ( op_mode_[id] == mode ) return true; // 既に同じモードの場合は何もしない
    if ( fabs((when_op_mode_updated_[id] - Time::now()).toSec()) < 1.0 ) ros::Duration(1.0).sleep(); // 1秒以内に変更した場合は1秒待つ
    /* トルクを一旦切る */ WriteTorqueEnable(id, TORQUE_DISABLE);
    /* モードを変更 */    WriteOperatingMode(id, mode);  // Operating Modeを変えると，RAM値が全部デフォルトに戻るっぽい．
    /* トルクを戻す */    WriteTorqueEnable(id, after);  // 動作中にこの関数が呼ばれることも考えて, TorqueOnは使わない
    // 結果を確認
    bool is_changed = mode == ReadOperatingMode(id);
    if ( is_changed ) {
        op_mode_[id] = mode;
        when_op_mode_updated_[id] = Time::now();
        ROS_INFO("ID [%d] is changed operating mode [%d]", id, mode);
    } else {
        ROS_ERROR("ID [%d] failed to change operating mode", id); 
    }
    return is_changed;
}

// モータの動作を停止させる．
bool DynamixelHandler::StopRotation(uint8_t id){
    if ( series_[id] != SERIES_X ) return false; // Xシリーズ以外は対応していない
    auto now_pos = ReadPresentPosition(id);
    cmd_values_[id][GOAL_POSITION]      = now_pos;
    state_values_[id][PRESENT_POSITION] = now_pos;
    auto cur_lim = option_limit_[id][CURRENT_LIMIT];
    auto now_cur = cmd_values_[id][GOAL_CURRENT];
    auto cur = clamp(now_cur, -cur_lim/5, cur_lim/5); // ストールトルクの20%まで制限
    if ( !WriteGoalPosition(id, now_pos)) return false; 
    if ( !WriteGoalVelocity(id, 0.0)    ) return false;
    if ( !WriteGoalCurrent (id, cur    )) return false;
    return true;
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

bool DynamixelHandler::WriteGoalVelocity(uint8_t id, double vel){
    auto vel_pulse = goal_velocity.val2pulse(vel, model_[id]);
    return dyn_comm_.tryWrite(goal_velocity, id, vel_pulse);
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

//* Main loop 内で使う全モータへの一括読み書き関数たち

/**
 * @func SyncWriteCommandValues
 * @brief 指定した範囲のコマンド値を書き込む
 * @param list_write_cmd 書き込むコマンドのEnumのリスト
*/
void DynamixelHandler::SyncWriteCommandValues(set<CmdValueIndex>& list_write_cmd){
    // 空なら即時return
    if ( list_write_cmd.empty() ) return;
    // 書き込む範囲のイテレータを取得
    const auto it_start = min_element(list_write_cmd.begin(), list_write_cmd.end());
          auto it_end   = max_element(list_write_cmd.begin(), list_write_cmd.end());
    // 分割書き込みが有効な場合を再帰で実装するため, 書き込む範囲を1つ目のみに制限
    if ( use_split_write_ ) it_end = it_start;
    // 書き込む範囲の値(cmd_dp_listのIndex)を取得, ここで読み取らないとeraseで消えてしまう
    const CmdValueIndex cmd_start = *it_start, cmd_end = *it_end;
    // 今回で書き込む範囲は削除, use_split_write_=falseの場合は全て削除, trueの場合は先頭だけ削除
    list_write_cmd.erase(it_start, ++it_end);
    // 書き込みに必要な変数を用意
    vector<DynamixelAddress> target_cmd_dp_list;  // 書き込むコマンドのアドレスのベクタ
    map<uint8_t, vector<int64_t>> id_cmd_vec_map; // id と 書き込むデータのベクタのマップ
    for (size_t cmd = cmd_start; cmd <= cmd_end; cmd++) { // アドレスのベクタと，データのベクタの並びは対応している必要があるので，同一のループで作成する．
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
    // 分割書き込みのための再帰的な処理, list_write_cmdが空ならすぐ帰ってくる
    SyncWriteCommandValues(list_write_cmd);
    // 後処理，再帰の終端で実行される
    is_cmd_updated_.clear();
}

/**
 * @func SyncReadStateValues
 * @brief 指定した範囲の状態値を読み込む
 * @param list_read_state 読み込む状態値のEnumのリスト
 * @return 読み込みに成功したかどうか
*/
bool DynamixelHandler::SyncReadStateValues(StValueIndex target){ set<StValueIndex> t = {target} ; return SyncReadStateValues(t);}
bool DynamixelHandler::SyncReadStateValues(const set<StValueIndex>& list_read_state){
    if ( list_read_state.empty() ) return false; // 空なら何もしない
    const StValueIndex start = *min_element(list_read_state.begin(), list_read_state.end());
    const StValueIndex end   = *max_element(list_read_state.begin(), list_read_state.end());
    if ( !(0 <= start && start <= end && end < state_dp_list.size()) ) return false;

    vector<DynamixelAddress> target_state_dp_list(
        state_dp_list.begin()+start, state_dp_list.begin()+end+1 );
    
    vector<uint8_t> target_id_list;
    for (auto id : id_list_) if ( series_[id]==SERIES_X ) target_id_list.push_back(id);

    auto id_st_vec_map = (use_fast_read_) //  fast readを使う設定の場合はfast readを使う 途中で切り替えるとtimeout後に来るデータによってSyncReadが何度も失敗するので注意
        ? dyn_comm_.SyncRead_fast(target_state_dp_list, target_id_list)
        : dyn_comm_.SyncRead     (target_state_dp_list, target_id_list);
    is_timeout_read_state_     = dyn_comm_.timeout_last_read();
    has_comm_error_read_state_ = dyn_comm_.comm_error_last_read();
    has_any_hardware_error_    = dyn_comm_.hardware_error_last_read();

    // 通信エラーの表示
    if ( varbose_read_st_err_ ) if ( has_comm_error_read_state_ || is_timeout_read_state_ ) {
        vector<uint8_t> failed_id_list;
        for ( auto id : target_id_list ) if ( id_st_vec_map.find(id) == id_st_vec_map.end() ) failed_id_list.push_back(id);
        char header[100]; sprintf(header, "[%d] servo(s) failed to read", (int)(target_id_list.size() - id_st_vec_map.size()));
        auto ss = id_list_layout(failed_id_list, string(header) + (is_timeout_read_state_? " (time out)" : " (some kind packet error)"));
        ROS_WARN_STREAM(ss);
    }
    // id_st_vec_mapの中身を確認
    if ( varbose_read_st_ ) if ( id_st_vec_map.size()>0 ) {
        char header[100]; sprintf(header, "[%d] servo(s) are read", (int)id_st_vec_map.size());
        auto ss = control_table_layout(width_log_, id_st_vec_map, target_state_dp_list, string(header));
        ROS_INFO_STREAM(ss);
        if (has_any_hardware_error_) ROS_WARN("Hardware Error are detected");
    }
    // state_values_に反映
    for (int i = 0; i <= end-start; i++) {
        const auto dp = target_state_dp_list[i];
        for (auto pair : id_st_vec_map) {
            const uint8_t id = pair.first;
            const int64_t data_int = pair.second[i];
            state_values_[id][start+i] = dp.pulse2val( data_int, model_[id]);
        }
    }

    return id_st_vec_map.size()>0; // 1つでも成功したら成功とする.
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

/**
 * @func SyncReadHardwareError
 * @brief ハードウェアエラーを読み込む
 * @return 読み込みに成功したかどうか
*/
bool DynamixelHandler::SyncReadHardwareErrors(){
    if ( !has_any_hardware_error_ ) {hardware_error_.clear(); return true;} // 事前にエラーが検出できていない場合は省略

    vector<uint8_t> target_id_list;
    for (int id : id_list_) if ( series_[id]==SERIES_X ) target_id_list.push_back(id);
    
    auto id_error_map = dyn_comm_.SyncRead(hardware_error_status, target_id_list);
    // if (dyn_comm_.timeout_last_read()   ) return false; // 読み込み失敗
    // if (dyn_comm_.comm_error_last_read()) return false; // 読み込み失敗

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
    return true;
}

bool DynamixelHandler::SyncReadOption_Mode(){
    return false;
}

bool DynamixelHandler::SyncReadOption_Gain(){
    return false;
}

bool DynamixelHandler::SyncReadOption_Limit(){
    vector<uint8_t> target_id_list;
    for (int id : id_list_) if ( series_[id]==SERIES_X ) target_id_list.push_back(id);

    auto id_limit_vec_map = dyn_comm_.SyncRead_fast(opt_limit_dp_list, target_id_list);   
    bool is_timeout_read     = dyn_comm_.timeout_last_read();
    bool has_comm_error_read = dyn_comm_.comm_error_last_read();

    // 通信エラーの表示
    if ( varbose_read_opt_err_ ) if ( has_comm_error_read || is_timeout_read ) {
        vector<uint8_t> failed_id_list;
        for ( auto id : target_id_list ) if ( id_limit_vec_map.find(id) == id_limit_vec_map.end() ) failed_id_list.push_back(id);
        char header[100]; sprintf(header, "[%d] servo(s) failed to read", (int)(target_id_list.size() - id_limit_vec_map.size()));
        auto ss = id_list_layout(failed_id_list, string(header) + (is_timeout_read_state_? " (time out)" : " (some kind packet error)"));
        ROS_WARN_STREAM(ss);
    }
    // id_limit_vec_mapの中身を確認
    if ( varbose_read_opt_ ) if ( id_limit_vec_map.size()>0 ) {
        char header[100]; sprintf(header, "[%d] servo(s) are read", (int)id_limit_vec_map.size());
        auto ss = control_table_layout(width_log_, id_limit_vec_map, opt_limit_dp_list, string(header));
        ROS_INFO_STREAM(ss);
    }

    // option_limit_に反映
    for ( auto opt_lim=0; opt_lim<opt_limit_dp_list.size(); opt_lim++) {
        DynamixelAddress dp = opt_limit_dp_list[opt_lim];
        for (auto pair : id_limit_vec_map) {
            const uint8_t id = pair.first;
            const int64_t data_int = pair.second[opt_lim];
            option_limit_[id][opt_lim] = dp.pulse2val( data_int, model_[id]);
        }
    }

    return id_limit_vec_map.size()>0; // 1つでも成功したら成功とする.
}

//* ROS関係

void DynamixelHandler::CallBackDxlCommandFree(const dynamixel_handler::DynamixelCommandFree& msg) {
    auto id_list = msg.id_list; 
    if (id_list.empty()) for (auto id : id_list_) id_list.push_back(id);
    if (msg.command == "clear")
        for (auto id : id_list) ClearHardwareError(id);
    if (msg.command == "enable") 
        for (auto id : id_list) TorqueOn(id);
    if (msg.command == "disable")
        for (auto id : id_list) TorqueOff(id);
    if (msg.command == "reboot") 
        for (auto id : id_list) dyn_comm_.Reboot(id);
}

void DynamixelHandler::CallBackDxlCommand_Profile(const dynamixel_handler::DynamixelCommand_Profile& msg) {

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
        ChangeOperatingMode(id, OPERATING_MODE_POSITION);
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
            cmd_values_[id][GOAL_CURRENT] = clamp(cur, -limit[CURRENT_LIMIT], limit[CURRENT_LIMIT]);
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

void DynamixelHandler::BroadcastDxlStateFree(){

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
