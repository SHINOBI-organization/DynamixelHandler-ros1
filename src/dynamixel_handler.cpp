#include "dynamixel_handler.hpp"

// 各シリーズのDynamixelを検出する．
uint8_t DynamixelHandler::ScanDynamixels(uint8_t id_max, uint8_t num_try, uint8_t wait_time_ms) {   
    id_list_x_.clear();
    id_list_p_.clear();

    for (uint8_t id = 0; id <= id_max; id++)
        for (size_t i = 0; i < num_try; i++) {
            bool is_found = dyn_comm_.Ping(id);
            sleep_for(1ms*wait_time_ms);
            if ( !is_found ) continue;
            int dyn_model = dyn_comm_.Read(model_number, id);
            if ( is_x_series(dyn_model) ) { 
                ROS_INFO(" * X series servo id [%d] is found", id);
                id_list_x_.push_back(id++); i=0; // 見つかったid残りの検査をスキップ
            } else if ( is_p_series(dyn_model) ) { 
                ROS_INFO(" * P series servo id [%d] is found", id);
                id_list_p_.push_back(id++); i=0; // 見つかったid残りの検査をスキップ
            } else { 
                ROS_WARN(" * Unkwon model [%d] servo id [%d] is found", dyn_model, id);
        }
    }
    return id_list_x_.size() + id_list_p_.size();
}

bool DynamixelHandler::ClearError(uint8_t id, DynamixelTorquePermission after_state){
    auto present_pos = dyn_comm_.Read(present_position, id);
    int present_rotation = present_pos / 2048; // 整数値に丸める //todo ここら辺の変換を自動でやる必要がある
    if (present_pos < 0) present_rotation--;

        dyn_comm_.Reboot(id);
        sleep_for(0.5s);

    dyn_comm_.Write(homing_offset, id, present_rotation * 2048);
    dyn_comm_.Write(torque_enable, id, after_state);
    return dyn_comm_.Read(hardware_error_status, id) == 0; // 0 is no error
}

bool DynamixelHandler::TorqueEnable(uint8_t id){
    dyn_comm_.Write(torque_enable, id, TORQUE_ENABLE);
    sleep_for(0.005s);
    return dyn_comm_.Read(torque_enable, id) != TORQUE_ENABLE;
}

bool DynamixelHandler::TorqueDisable(uint8_t id){
    dyn_comm_.Write(torque_enable, id, TORQUE_DISABLE);
    sleep_for(0.005s);
    return dyn_comm_.Read(torque_enable, id) != TORQUE_DISABLE;
}

void DynamixelHandler::SyncWritePosition(){
    vector<int64_t> data_int_list(id_list_x_.size());
    for (size_t i = 0; i < id_list_x_.size(); i++) data_int_list[i] = dynamixel_chain[id_list_x_[i]].goal_position;
    dyn_comm_.SyncWrite(goal_position, id_list_x_, data_int_list);
}

bool DynamixelHandler::SyncReadPosition(){
    auto id_data_map = (has_hardware_error)
        ? dyn_comm_.SyncRead     (present_position, id_list_x_)  // エラーがあるときは遅い方
        : dyn_comm_.SyncRead_fast(present_position, id_list_x_); // エラーがあるときは早い方
    // エラー処理
    if (id_data_map.size() < id_list_x_.size()){
        ROS_WARN("SyncReadPosition: %d servo(s) failed to read", (int)(id_list_x_.size() - id_data_map.size()));
        for ( auto id : id_list_x_ )
            if ( id_data_map.find(id) == id_data_map.end() )
                ROS_WARN("  * servo id [%d] failed to read", id);
    }
    // 読み込みに成功したデータをdynamixel_chainに反映
    for (auto id_pos : id_data_map) dynamixel_chain[id_pos.first].present_position = id_pos.second; 
    return id_data_map.size()>0; // 1つでも成功したら成功とする.
}

void DynamixelHandler::SyncReadHardwareError(){
    ROS_INFO("SyncReadHardwareInfo: Checking hardware error");
    
    auto id_error_map = dyn_comm_.SyncRead(hardware_error_status, id_list_x_);

    has_hardware_error = false;
    for (auto pair : id_error_map) has_hardware_error += (bool)pair.second; // errorがあるときは0以外の値になる．
    if ( !has_hardware_error ) return;

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
}

void DynamixelHandler::ShowDynamixelChain(){
    // dynamixel_chainのすべての内容を表示
    for (auto id : id_list_x_) {
        ROS_INFO("== Servo id [%d] ==", id);
        ROS_INFO("  present_position [%d] pulse", dynamixel_chain[id].present_position);
        ROS_INFO("  goal_position    [%d] pulse", dynamixel_chain[id].goal_position);
    }
}

void DynamixelHandler::CallBackOfDynamixelCommand(const dynamixel_handler::DynamixelCmd& msg) {
    if (msg.command == "show") ShowDynamixelChain();
    if (msg.command == "reboot") { 
        for (auto id : msg.ids) ClearError(id);
        if (msg.ids.size() == 0) for (auto id : id_list_x_) ClearError(id);
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