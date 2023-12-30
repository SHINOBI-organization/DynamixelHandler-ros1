#include "dynamixel_handler.hpp"

using namespace dyn_x;

// 各シリーズのDynamixelを検出する．
uint8_t DynamixelHandler::ScanDynamixels(uint8_t id_max) {   
    id_list_.clear();

    for (int id = 0; id <= id_max; id++) {
        if ( !dyn_comm_.tryPing(id) ) continue;
        int dyn_model = dyn_comm_.tryRead(model_number, id);
        switch ( dynamixel_series(dyn_model) ) { 
            case SERIES_X: ROS_INFO(" * X series servo id [%d] is found", id);
                model_[id] = dyn_model;
                series_[id] = SERIES_X;
                id_list_.push_back(id); break;
            case SERIES_P: ROS_INFO(" * P series servo id [%d] is found", id);
                model_[id] = dyn_model;
                series_[id] = SERIES_P;
                id_list_.push_back(id); break;
            default: ROS_WARN(" * Unkwon model [%d] servo id [%d] is found", dyn_model, id);
        }
    }
    return id_list_.size();
}

bool DynamixelHandler::ClearHardwareError(uint8_t id, DynamixelTorquePermission after_state){
    if ( dyn_comm_.tryRead(hardware_error_status, id) == 0b00000000 ) return true; // エラーがない場合は何もしない

    auto now_pos_pulse = dyn_comm_.tryRead(present_position, id);
    auto now_pos_rad   = present_position.pulse2val(now_pos_pulse, model_[id]);
    int now_rot = (now_pos_rad+M_PI) / (2*M_PI);
    if (now_pos_rad < -M_PI) now_rot--;

    dyn_comm_.Reboot(id);
    sleep_for(0.5s);

    auto offset_pulse = homing_offset.val2pulse(now_rot*(2*M_PI), model_[id]);
    dyn_comm_.tryWrite(homing_offset, id, offset_pulse);

    after_state==TORQUE_ENABLE ? TorqueEnable(id) : TorqueDisable(id);

    bool is_clear = dyn_comm_.tryRead(hardware_error_status, id) == 0b00000000;
    if (is_clear) ROS_INFO ("Servo id [%d] is cleared error", id);
    else          ROS_ERROR("Servo id [%d] failed to clear error", id);
    return is_clear;
}

bool DynamixelHandler::TorqueEnable(uint8_t id){
    // 角度の同期
    int now_pos = dyn_comm_.tryRead(present_position, id);
    cmd_values_[id][GOAL_POSITION]      = present_position.pulse2val(now_pos, model_[id]);
    state_values_[id][PRESENT_POSITION] = present_position.pulse2val(now_pos, model_[id]);
    dyn_comm_.tryWrite(goal_position, id, now_pos);
    // トルクオン
    dyn_comm_.tryWrite(torque_enable, id, TORQUE_ENABLE);
    return dyn_comm_.tryRead(torque_enable, id) == TORQUE_ENABLE;
}

bool DynamixelHandler::TorqueDisable(uint8_t id){
    dyn_comm_.tryWrite(torque_enable, id, TORQUE_DISABLE);
    return dyn_comm_.tryRead(torque_enable, id) == TORQUE_DISABLE;
}

/**
 * @func SyncWriteCmdValues
 * @brief 指定した範囲のコマンド値を書き込む
 * @param list_wirte_cmd 書き込むコマンドのEnumのリスト
*/
void DynamixelHandler::SyncWriteCmdValues(CmdValues target){ set<CmdValues> t = {target} ; return SyncWriteCmdValues(t);} 
void DynamixelHandler::SyncWriteCmdValues(const set<CmdValues>& list_wirte_cmd){
    if ( list_wirte_cmd.empty() ) return; // 空なら何もしない
    const CmdValues start = *min_element(list_wirte_cmd.begin(), list_wirte_cmd.end());
    const CmdValues end   = *max_element(list_wirte_cmd.begin(), list_wirte_cmd.end());
    if ( !(0 <= start && start <= end && end < cmd_dp_list.size()) ) return;

    vector<DynamixelAddress> target_cmd_dp_list;   // 書き込むコマンドのアドレスのベクタを作成
    map<uint8_t, vector<int64_t>> id_data_vec_map; // id と 書き込むデータのベクタのマップを作成
    // アドレスのベクタと，データのベクタの並びは対応している必要があるので，同一のループで作成する．
    for (int cmd = start; cmd <= end; cmd++) {
        const auto dp = cmd_dp_list[cmd];
        target_cmd_dp_list.push_back(dp); 
        for (auto id : id_list_) if ( series_[id]==SERIES_X ) {
            if ( !is_cmd_updated_[id] ) continue; // 更新されていない場合はスキップ
            id_data_vec_map[id].push_back( dp.val2pulse( cmd_values_[id][cmd], model_[id]) );
        }
    }

    //id_data_vec_mapの中身を確認
    if ( varbose_write_cmd_ ) {
        ROS_INFO("SyncWriteCmdValues: %d servo(s) will be written", (int)id_data_vec_map.size());
        for ( auto pair : id_data_vec_map ) {
            ROS_INFO("  * servo id [%d] will be written", pair.first);
            for ( auto data : pair.second ) ROS_INFO("    - %d", (int)data);
        }
    }
    dyn_comm_.SyncWrite(target_cmd_dp_list, id_data_vec_map);
}

/**
 * @func SyncReadStateValues
 * @brief 指定した範囲の状態値を読み込む
 * @param list_read_state 読み込む状態値のEnumのリスト
 * @return 読み込みに成功したかどうか
*/
bool DynamixelHandler::SyncReadStateValues(StateValues target){ set<StateValues> t = {target} ; return SyncReadStateValues(t);}
bool DynamixelHandler::SyncReadStateValues(const set<StateValues>& list_read_state){
    if ( list_read_state.empty() ) return false; // 空なら何もしない
    const StateValues start = *min_element(list_read_state.begin(), list_read_state.end());
    const StateValues end   = *max_element(list_read_state.begin(), list_read_state.end());
    if ( !(0 <= start && start <= end && end < state_dp_list.size()) ) return false;

    vector<DynamixelAddress> target_state_dp_list(
        state_dp_list.begin()+start, state_dp_list.begin()+end+1 );
    
    vector<uint8_t> target_id_list;
    for (auto id : id_list_) if ( series_[id]==SERIES_X ) target_id_list.push_back(id);

    auto id_data_vec_map = (use_fast_read_&& !is_timeout_read_state_) //  fast readを使う設定かつ，直前でタイムアウトしていない場合はfast readを使う
        ? dyn_comm_.SyncRead_fast(target_state_dp_list, target_id_list)
        : dyn_comm_.SyncRead     (target_state_dp_list, target_id_list);
    is_timeout_read_state_     = dyn_comm_.timeout_last_read();
    has_comm_error_read_state_ = dyn_comm_.comm_error_last_read();
    has_any_hardware_error_    = dyn_comm_.hardware_error_last_read();

    // 通信エラーの表示
    if ( varbose_read_st_err_ ) if ( has_comm_error_read_state_ || is_timeout_read_state_ ) {
        ROS_WARN("SyncReadStateValues: %d servo(s) failed to read", (int)(target_id_list.size() - id_data_vec_map.size()));
        for ( auto id : target_id_list )
            if ( id_data_vec_map.find(id) == id_data_vec_map.end() ) 
                ROS_WARN("  * servo id [%d] failed to read", id);
    }
    // id_data_vec_mapの中身を確認
    if ( varbose_read_st_ ) {
        ROS_INFO("SyncReadStateValues: %d servo(s) are read", (int)id_data_vec_map.size());
        for ( auto pair : id_data_vec_map ) {
            ROS_INFO("  * servo id [%d] is read", pair.first);
            for ( auto data : pair.second ) ROS_INFO("    - %d", (int)data);
        }
        if (has_any_hardware_error_) ROS_WARN("SyncReadStateValues: Hardware Error are detected");
    }
    // state_values_に反映
    for (int i = 0; i <= end-start; i++) {
        const auto dp = target_state_dp_list[i];
        for (auto pair : id_data_vec_map) {
            const uint8_t id = pair.first;
            const int64_t data_int = pair.second[i];
            state_values_[id][start+i] = dp.pulse2val( data_int, model_[id]);
        }
    }

    return id_data_vec_map.size()>0; // 1つでも成功したら成功とする.
}

/**
 * @func SyncReadHardwareError
 * @brief ハードウェアエラーを読み込む
 * @return 読み込みに成功したかどうか
*/
bool DynamixelHandler::SyncReadHardwareError(){
    vector<uint8_t> target_id_list;
    for (int id : id_list_) if ( series_[id]==SERIES_X ) target_id_list.push_back(id);
    
    auto id_error_map = dyn_comm_.SyncRead(hardware_error_status, target_id_list);
    if (dyn_comm_.timeout_last_read()   ) return false; // 読み込み失敗
    if (dyn_comm_.comm_error_last_read()) return false; // 読み込み失敗

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
        ROS_WARN("SyncReadHardwareError: Hardware error are Checked");
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

bool DynamixelHandler::SyncReadConfigParameter_Mode(){
    return false;
}

bool DynamixelHandler::SyncReadConfigParameter_Gain(){
    return false;
}

bool DynamixelHandler::SyncReadConfigParameter_Limit(){
    return false;
}


void DynamixelHandler::CallBackDxlCommandFree(const dynamixel_handler::DynamixelCommandFree& msg) {
    auto id_list = msg.id_list; 
    if (id_list.empty()) for (auto id : id_list_) id_list.push_back(id);
    if (msg.command == "clear")
        for (auto id : id_list) ClearHardwareError(id);
    if (msg.command == "enable") 
        for (auto id : id_list) TorqueEnable(id);
    if (msg.command == "disable")
        for (auto id : id_list) TorqueDisable(id);
    if (msg.command == "reboot") 
        for (auto id : id_list) dyn_comm_.Reboot(id);
}

void DynamixelHandler::CallBackDxlCommand_Option(const dynamixel_handler::DynamixelCommand_Option& msg) {

}

void DynamixelHandler::CallBackDxlCommand_X_Position(const dynamixel_handler::DynamixelCommand_X_ControlPosition& msg) {
    if (varbose_callback_) ROS_INFO("CallBackDxlCommand_X_Position"); // msg.id_listと同じサイズの奴だけ処理する
    if ( msg.id_list.size() != msg.position__deg.size() ) { ROS_ERROR(" - Element size dismatch"); return;}

    for (int i = 0; i < msg.id_list.size(); i++) {
        int id = msg.id_list[i];
        auto pos = msg.position__deg[i];
        cmd_values_[id][GOAL_POSITION] = deg2rad(pos);
        is_cmd_updated_[id] = true;
        list_wirte_cmd_.insert(GOAL_POSITION);
    }
    if (varbose_callback_) ROS_INFO(" -  %d servo(s) goal_position are updated", (int)msg.id_list.size());
}

void DynamixelHandler::CallBackDxlCommand_X_Velocity(const dynamixel_handler::DynamixelCommand_X_ControlVelocity& msg) {
    if (varbose_callback_) ROS_INFO("CallBackDxlCommand_X_Velocity"); // msg.id_listと同じサイズの奴だけ処理する
    if ( msg.id_list.size() != msg.velocity__deg_s.size() ) { ROS_ERROR(" - Element size dismatch"); return;}

    for (int i = 0; i < msg.id_list.size(); i++) {
        int id = msg.id_list[i];
        auto vel = msg.velocity__deg_s[i];
        cmd_values_[id][GOAL_VELOCITY] = deg2rad(vel);
        is_cmd_updated_[id] = true;
        list_wirte_cmd_.insert(GOAL_VELOCITY);
    }
    if (varbose_callback_) ROS_INFO(" -  %d servo(s) goal_velocity are updated", (int)msg.id_list.size());
}

void DynamixelHandler::CallBackDxlCommand_X_Current(const dynamixel_handler::DynamixelCommand_X_ControlCurrent& msg) {
    if (varbose_callback_) ROS_INFO("CallBackDxlCommand_X_Current"); // msg.id_listと同じサイズの奴だけ処理する
    if ( msg.id_list.size() != msg.current__mA.size() ) { ROS_ERROR(" - Element size dismatch"); return;}

    for (int i = 0; i < msg.id_list.size(); i++) {
        int id = msg.id_list[i];
        auto cur = msg.current__mA[i];
        cmd_values_[id][GOAL_CURRENT] = cur;
        is_cmd_updated_[id] = true;
        list_wirte_cmd_.insert(GOAL_CURRENT);
    }
    if (varbose_callback_) ROS_INFO(" -  %d servo(s) goal_current are updated", (int)msg.id_list.size());
}

void DynamixelHandler::CallBackDxlCommand_X_CurrentPosition(const dynamixel_handler::DynamixelCommand_X_ControlCurrentPosition& msg) {
    if (varbose_callback_) ROS_INFO("CallBackDxlCommand_X_CurrentPosition"); // msg.id_listと同じサイズの奴だけ処理する
    const bool do_process_cur = msg.id_list.size() == msg.current__mA.size();
    const bool do_process_pos = msg.id_list.size() == msg.position__deg.size() || msg.id_list.size() == msg.rotation.size();
    if ( !do_process_cur && !do_process_pos ) { ROS_ERROR(" - Element size dismatch"); return;}

    for (int i = 0; i < msg.id_list.size(); i++) {
        int id = msg.id_list[i];
        if ( do_process_pos ){
            auto pos = msg.id_list.size() == msg.position__deg.size() ? msg.position__deg[i] : 0.0;
            auto rot = msg.id_list.size() == msg.rotation.size()      ? msg.rotation[i]      : 0;
            is_cmd_updated_[id] = true;
            cmd_values_[id][GOAL_POSITION] = deg2rad(pos + rot * 360.0);
            list_wirte_cmd_.insert(GOAL_POSITION);
        }
        if ( do_process_cur ){
            auto cur = msg.current__mA[i];
            is_cmd_updated_[id] = true;
            cmd_values_[id][GOAL_CURRENT] = cur;
            list_wirte_cmd_.insert(GOAL_CURRENT);
        }
    }
    if (varbose_callback_ && do_process_cur) ROS_INFO(" -  %d servo(s) goal_current are updated", (int)msg.id_list.size());
    if (varbose_callback_ && do_process_pos) ROS_INFO(" -  %d servo(s) goal_position are updated", (int)msg.id_list.size());
}

void DynamixelHandler::CallBackDxlCommand_X_ExtendedPosition(const dynamixel_handler::DynamixelCommand_X_ControlExtendedPosition& msg) {
    if (varbose_callback_) ROS_INFO("CallBackDxlCommand_X_ExtendedPosition");
    const bool do_process = msg.id_list.size() == msg.position__deg.size() || msg.id_list.size() == msg.rotation.size();
    if ( !do_process ) { ROS_ERROR(" - Element size dismatch"); return;}

    for (int i = 0; i < msg.id_list.size(); i++) {
        int id = msg.id_list[i];
        auto pos = msg.id_list.size() == msg.position__deg.size() ? msg.position__deg[i] : 0.0;
        auto rot = msg.id_list.size() == msg.rotation.size()      ? msg.rotation[i]      : 0;
        is_cmd_updated_[id] = true;
        cmd_values_[id][GOAL_POSITION] = deg2rad(pos + rot * 360.0);
        list_wirte_cmd_.insert(GOAL_POSITION);
    }
    if (varbose_callback_) ROS_INFO(" -  %d servo(s) goal_position are updated", (int)msg.id_list.size());
}

void DynamixelHandler::BroadcastDxlStateFree(){

}

void DynamixelHandler::BroadcastDxlState(){
    dynamixel_handler::DynamixelState msg;
    msg.stamp = ros::Time::now();
    for (auto id : id_list_) {
        msg.id_list.push_back(id);
        for (auto state : list_read_state_) switch(state) {
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

void DynamixelHandler::BroadcastDxlConfig_Limit(){

}
void DynamixelHandler::BroadcastDxlConfig_Gain(){

}
void DynamixelHandler::BroadcastDxlConfig_Mode(){

}
