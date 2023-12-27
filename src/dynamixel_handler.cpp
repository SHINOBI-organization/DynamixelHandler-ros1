#include "dynamixel_handler.hpp"

using namespace dyn_x;

// 各シリーズのDynamixelを検出する．
uint8_t DynamixelHandler::ScanDynamixels(uint8_t id_max) {   
    id_list_x_.clear();
    id_list_p_.clear();

    for (int id = 0; id <= id_max; id++) {
        if ( !dyn_comm_.tryPing(id) ) continue;
        int dyn_model = dyn_comm_.tryRead(model_number, id);
        if ( is_x_series(dyn_model) ) { 
            ROS_INFO(" * X series servo id [%d] is found", id);
            id_list_x_.push_back(id); // 見つかったid残りの検査をスキップ
        } else if ( is_p_series(dyn_model) ) { 
            ROS_INFO(" * P series servo id [%d] is found", id);
            id_list_p_.push_back(id); // 見つかったid残りの検査をスキップ
        } else { 
            ROS_WARN(" * Unkwon model [%d] servo id [%d] is found", dyn_model, id);
        }
    }
    return id_list_x_.size() + id_list_p_.size();
}

bool DynamixelHandler::ClearHardwareError(uint8_t id, DynamixelTorquePermission after_state){
    auto present_pos = pulse2rad(dyn_comm_.tryRead(present_position, id)) ;
    int present_rotation = present_pos / (2*M_PI); // 整数値に丸める //todo ここら辺の変換を自動でやる必要がある
    if (present_pos < 0) present_rotation--;

    dyn_comm_.Reboot(id);
    sleep_for(0.5s);

    dyn_comm_.tryWrite(homing_offset, id, rad2pulse((present_rotation * M_PI)));
    after_state==TORQUE_ENABLE ? TorqueEnable(id) : TorqueDisable(id);

    if (dyn_comm_.tryRead(hardware_error_status, id) == 0b00000000) {
        ROS_INFO("Servo id [%d] is cleared error", id);
        return true;
    } else {
        ROS_ERROR("Servo id [%d] failed to clear error", id);
        return false;
    }
}

bool DynamixelHandler::TorqueEnable(uint8_t id){
    // 角度の同期
    int present_pos = dyn_comm_.tryRead(present_position, id);
    cmd_values_[id][GOAL_POSITION]      = present_pos;
    state_values_[id][PRESENT_POSITION] = present_pos;
    dyn_comm_.tryWrite(goal_position, id, present_pos);
    // トルクオン
    dyn_comm_.tryWrite(torque_enable, id, TORQUE_ENABLE);
    return dyn_comm_.tryRead(torque_enable, id) != TORQUE_ENABLE;
}

bool DynamixelHandler::TorqueDisable(uint8_t id){
    dyn_comm_.tryWrite(torque_enable, id, TORQUE_DISABLE);
    return dyn_comm_.tryRead(torque_enable, id) != TORQUE_DISABLE;
}

bool DynamixelHandler::CheckHardwareError(uint8_t id){
    return dyn_comm_.tryRead(hardware_error_status, id); //todo dyn_comm_にエラー簡易的なHWエラー判定を実装する
}

bool DynamixelHandler::SyncReadHardwareError(){
    ROS_INFO("SyncReadHardwareInfo: Checking hardware error");
    
    auto id_error_map = dyn_comm_.SyncRead(hardware_error_status, id_list_x_);

    has_hardware_error = false;
    for (auto pair : id_error_map) has_hardware_error += (bool)pair.second; // errorがあるときは0以外の値になる．
    if ( !has_hardware_error ) return false;

    ROS_ERROR("SyncReadHardwareError: Hardware Error is detected");
    for (auto pair : id_error_map) {
        uint8_t id = pair.first;
        uint8_t error = pair.second;
        if ((error >> HARDWARE_ERROR_INPUT_VOLTAGE)     & 0b1 ) ROS_ERROR("  * servo id [%d] : INPUT_VOLTAGE",     id);
        if ((error >> HARDWARE_ERROR_MOTOR_HALL_SENSOR) & 0b1 ) ROS_ERROR("  * servo id [%d] : MOTOR_HALL_SENSOR", id);
        if ((error >> HARDWARE_ERROR_OVERHEATING)       & 0b1 ) ROS_ERROR("  * servo id [%d] : OVERHEATING",       id);
        if ((error >> HARDWARE_ERROR_MOTOR_ENCODER)     & 0b1 ) ROS_ERROR("  * servo id [%d] : MOTOR_ENCODER",     id);
        if ((error >> HARDWARE_ERROR_ELECTRONICAL_SHOCK)& 0b1 ) ROS_ERROR("  * servo id [%d] : ELECTRONICAL_SHOCK",id);
        if ((error >> HARDWARE_ERROR_OVERLOAD)          & 0b1 ) ROS_ERROR("  * servo id [%d] : OVERLOAD",          id);
    }
    return true;
}

void DynamixelHandler::CallBackOfDynamixelCommand(const dynamixel_handler::DynamixelCmdFree& msg) {
    // if (msg.command == "reboot") { 
    //     for (auto id : msg.ids) ClearHardwareError(id);
    //     if (msg.ids.size() == 0) for (auto id : id_list_x_) ClearHardwareError(id);
    // }
    // if (msg.command == "write") {

    // }
}

void DynamixelHandler::CallBackOfDxlCmd_X_Position(const dynamixel_handler::DynamixelCmd_X_ControlPosition& msg) {
    if ( msg.id_list.size() != msg.position__deg.size() ) return;

    for (int i = 0; i < msg.id_list.size(); i++) {
        int id = msg.id_list[i];
        auto pos = msg.position__deg[i];
        cmd_values_[id][GOAL_POSITION] = pos;
        is_updated_[id] = true;
        list_wirte_cmd_.insert(GOAL_POSITION);
    }
}

void DynamixelHandler::CallBackOfDxlCmd_X_Velocity(const dynamixel_handler::DynamixelCmd_X_ControlVelocity& msg) {
    if ( msg.id_list.size() != msg.velocity__deg_s.size() ) return;

    for (int i = 0; i < msg.id_list.size(); i++) {
        int id = msg.id_list[i];
        auto vel = msg.velocity__deg_s[i];
        cmd_values_[id][GOAL_VOLOCITY] = vel;
        is_updated_[id] = true;
        list_wirte_cmd_.insert(GOAL_VOLOCITY);
    }
}

void DynamixelHandler::CallBackOfDxlCmd_X_Current(const dynamixel_handler::DynamixelCmd_X_ControlCurrent& msg) {
    if ( msg.id_list.size() != msg.current__mA.size() ) return;

    for (int i = 0; i < msg.id_list.size(); i++) {
        int id = msg.id_list[i];
        auto cur = msg.current__mA[i];
        cmd_values_[id][GOAL_CURRENT] = cur;
        is_updated_[id] = true;
        list_wirte_cmd_.insert(GOAL_CURRENT);
    }
}

void DynamixelHandler::CallBackOfDxlCmd_X_CurrentPosition(const dynamixel_handler::DynamixelCmd_X_ControlCurrentPosition& msg) {
    if ( !msg.current__mA.empty()   && msg.id_list.size() != msg.current__mA.size()  ) return; // 空なら無視するのでOK, 空でないならサイズ一致が必要
    if ( !msg.position__deg.empty() && msg.id_list.size() != msg.position__deg.size()) return; // なので，空でなくかつサイズ不一致はスキップする
    if ( !msg.rotation.empty()      && msg.id_list.size() != msg.rotation.size()     ) return;

    for (int i = 0; i < msg.id_list.size(); i++) {
        int id = msg.id_list[i];
        if (!msg.position__deg.empty() || !msg.rotation.empty()){
            auto pos = !msg.position__deg.empty() ? msg.position__deg[i] : cmd_values_[id][GOAL_POSITION] ;
            auto rot = !msg.rotation.empty()      ? msg.rotation[i]      : 0;
            is_updated_[id] = true;
            cmd_values_[id][GOAL_POSITION] = pos + rot * 360.0;
            list_wirte_cmd_.insert(GOAL_POSITION);
        }
        if (!msg.current__mA.empty()){
            auto cur = msg.current__mA[i];
            is_updated_[id] = true;
            cmd_values_[id][GOAL_CURRENT] = cur;
            list_wirte_cmd_.insert(GOAL_CURRENT);
        }
    }
}

void DynamixelHandler::CallBackOfDxlCmd_X_ExtendedPosition(const dynamixel_handler::DynamixelCmd_X_ControlExtendedPosition& msg) {
    if ( !msg.position__deg.empty() && msg.id_list.size() != msg.position__deg.size()) return; // 空なら無視するのでOK, 空でないならサイズ一致が必要
    if ( !msg.rotation.empty()     && msg.id_list.size() != msg.rotation.size()      ) return; // なので，空でなくかつサイズ不一致はスキップする

    for (int i = 0; i < msg.id_list.size(); i++) {
        int id = msg.id_list[i];
        auto pos = !msg.position__deg.empty() ? msg.position__deg[i] : cmd_values_[id][GOAL_POSITION] ;
        auto rot = !msg.rotation.empty()      ? msg.rotation[i]      : 0;
        is_updated_[id] = true;
        cmd_values_[id][GOAL_POSITION] = pos + rot * 360.0;
        list_wirte_cmd_.insert(GOAL_POSITION);
    }
}

/**
 * @func SyncWriteCmdValues
 * @brief 指定した範囲のコマンド値を書き込む
 * @param list_wirte_cmd 書き込むコマンドのEnumのリスト
*/
void DynamixelHandler::SyncWriteCmdValues(set<CmdValues>& list_wirte_cmd){
    const int start =     *min_element(list_wirte_cmd.begin(), list_wirte_cmd.end());
    const int end   = 1 + *max_element(list_wirte_cmd.begin(), list_wirte_cmd.end());
    list_wirte_cmd.clear(); // 先にクリアすることで，誤ってstartと　endに変な値が困窮した場合の影響が永続しないようにする．
    if ( !(0 <= start && start < end && end <= cmd_dp_list.size()) ) return;

    vector<DynamixelAddress> target_cmd_dp_list(
         cmd_dp_list.begin()+start, cmd_dp_list.begin()+end );
   
    map<uint8_t, vector<int64_t>> id_data_vec_map;
    for (int id : id_list_x_) if ( is_updated_[id] ) { 
        is_updated_[id] = false; // 更新済みフラグをリセット
        // 書き込むデータをCmdValuesの範囲を基に切り出す
    	vector<int64_t> data_int_list( // todo DynamixelAdderessの情報に合わせて， double -> int64_t に変換する
            cmd_values_[id].begin()+start, cmd_values_[id].begin()+end );
        id_data_vec_map[id] = data_int_list;
    }
    // dyn_comm_.SyncWrite(target_cmd_dp_list, id_data_vec_map);
}
void DynamixelHandler::SyncWriteCmdValues(CmdValues target){ 
    set<CmdValues> tmp{target};
    return SyncWriteCmdValues( tmp ); 
}

/**
 * @func SyncReadStateValues
 * @brief 指定した範囲の状態値を読み込む
 * @param list_read_state 読み込む状態値のEnumのリスト
 * @return 読み込みに成功したかどうか
*/
bool DynamixelHandler::SyncReadStateValues(set<StateValues>& list_read_state){
    const int start =     *min_element(list_read_state.begin(), list_read_state.end());
    const int end   = 1 + *max_element(list_read_state.begin(), list_read_state.end());
    list_read_state.clear(); // 先にクリアすることで，誤ってstartと　endに変な値が困窮した場合の影響が永続しないようにする．
    if ( !(0 <= start && start < end && end <= state_dp_list.size()) ) return false;

    vector<DynamixelAddress> target_state_dp_list(
        state_dp_list.begin()+start, state_dp_list.begin()+end );

    // target_state_dp_listの中身を確認
    for (auto dp : target_state_dp_list) ROS_INFO("dp: %d", (int)dp.address());

    return true;
    // use_fast_read_ ? dyn_comm_.SyncRead_fast(target_state_dp_list, id_list_x_)
    //                : dyn_comm_.SyncRead     (target_state_dp_list, id_list_x_);
    // // エラー処理
    // if (id_data_map.size() < id_list_x_.size()){
    //     ROS_WARN("SyncReadPosition: %d servo(s) failed to read", (int)(id_list_x_.size() - id_data_map.size()));
    //     for ( auto id : id_list_x_ )
    //         if ( id_data_map.find(id) == id_data_map.end() )
    //             ROS_WARN("  * servo id [%d] failed to read", id);
    // }

    // for (auto id_pos : id_data_map) {
    //     int id = id_pos.first;  // todo DynamixelAdderessの情報に合わせて， double -> int64_t に変換する
    //     for (int i = 0; i < id_pos.second.size(); i++) 
    //         state_vals[id][start+i] = id_pos.second[i];
    // }

    // return id_data_map.size()>0; // 1つでも成功したら成功とする.
}
bool DynamixelHandler::SyncReadStateValues(StateValues target){ 
    set<StateValues> tmp{target};
    return SyncReadStateValues( tmp ); 
}

void DynamixelHandler::BroadcastDynamixelState(){
    dynamixel_handler::DynamixelState msg;
    for (auto id : id_list_x_) {
        msg.id_list.push_back(id);
        for (auto state : list_read_state_) switch(state){
            case PRESENT_CURRENT:      msg.current__mA.push_back          (state_values_[id][PRESENT_CURRENT      ]); break;
            case PRESENT_VELOCITY:     msg.velocity__deg_s.push_back      (state_values_[id][PRESENT_VELOCITY     ]); break;
            case PRESENT_POSITION:     msg.position__deg.push_back        (state_values_[id][PRESENT_POSITION     ]); break;
            case VELOCITY_TRAJECTORY:  msg.vel_trajectory__deg_s.push_back(state_values_[id][VELOCITY_TRAJECTORY  ]); break;
            case POSITION_TRAJECTORY:  msg.pos_trajectory__deg.push_back  (state_values_[id][POSITION_TRAJECTORY  ]); break;
            case PRESENT_TEMPERTURE:   msg.temperature__degC.push_back    (state_values_[id][PRESENT_TEMPERTURE   ]); break;
            case PRESENT_INPUT_VOLTAGE:msg.input_voltage__V.push_back     (state_values_[id][PRESENT_INPUT_VOLTAGE]); break;
        }
    }
    pub_dyn_state_.publish(msg);
}