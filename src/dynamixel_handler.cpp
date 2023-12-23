#include "dynamixel_handler.hpp"

// ここら変の情報は型番固有の情報なので， dynamixel_parameter.hpp/cpp側に記述して，将来的には自動で読み込ませるようにしたい．
// int64_t deg2pulse(double deg) { return deg * 4096.0 / 360.0 + 2048; }
// double  pulse2deg(int64_t pulse) { return (pulse - 2048 ) * 360.0 / 4096.0; }
int64_t rad2pulse(double rad) { return rad * 4096.0 / (2.0 * M_PI) + 2048; }
double  pulse2rad(int64_t pulse) { return (pulse - 2048 ) * 2.0 * M_PI / 4096.0; }
// int64_t mA2pulse(double mA) { return mA / 1.0; }
// double  pulse2mA(int64_t pulse) { return pulse * 1.0; }

ros::Subscriber DynamixelHandler::sub_cmd_;
bool DynamixelHandler::varbose_;
int DynamixelHandler::loop_rate_;
int DynamixelHandler::error_ratio_;
DynamixelComunicator DynamixelHandler::dyn_comm_;
vector<uint8_t> DynamixelHandler::id_list_x_;
vector<uint8_t> DynamixelHandler::id_list_p_;
vector<DynamixelHandler::Dynamixel> DynamixelHandler::dynamixel_chain;
bool DynamixelHandler::is_updated;
bool DynamixelHandler::has_hardware_error;

bool DynamixelHandler::ScanDynamixels(uint8_t id_max) {   
    id_list_x_.clear();
    id_list_p_.clear();

    for (int id = 1; id <= id_max; id++) {
        bool is_found = false;
        for (size_t i = 0; i < 5; i++) if ( !is_found ) {
            if (dyn_comm_.Ping(id)) is_found = true;
            sleep_for(0.01s);  
        }
        if (is_found) {
            auto dyn_model = dyn_comm_.Read(id, model_number);
            id_list_x_.push_back(id); // とりあえず全てのサーボをx_seriesとして扱う
            // if ( is_x_series(dyn_model) ) id_list_x_.push_back(id);
            // if ( is_p_series(dyn_model) ) id_list_p_.push_back(id);
            printf(" * Servo id [%d] is found (id range 1 to [%d])\n", id, id_max);
        }
    }
    return id_list_x_.size() > 0 || id_list_p_.size() > 0;
}

bool DynamixelHandler::ClearError(uint8_t id, DynamixelTorquePermission after_state){
    auto present_pos = dyn_comm_.Read(id, present_position);
    int present_rotation = present_pos / 2048; // 整数値に丸める
    if (present_pos < 0) present_rotation--;

        dyn_comm_.Reboot(id);
        sleep_for(0.5s);

    dyn_comm_.Write(id, homing_offset, present_rotation * 2048);
    dyn_comm_.Write(id, torque_enable, after_state);
    return dyn_comm_.Read(id, hardware_error_status) == 0; // 0 is no error
}

bool DynamixelHandler::TorqueEnable(uint8_t id){
    dyn_comm_.Write(id, torque_enable, TORQUE_ENABLE);
    sleep_for(0.01s);
    return dyn_comm_.Read(id, torque_enable) != TORQUE_ENABLE;
}

bool DynamixelHandler::TorqueDisable(uint8_t id){
    dyn_comm_.Write(id, torque_enable, TORQUE_DISABLE);
    sleep_for(0.01s);
    return dyn_comm_.Read(id, torque_enable) != TORQUE_DISABLE;
}

void DynamixelHandler::SyncWritePosition(){
    vector<int64_t> data_int_list(id_list_x_.size());
    for (size_t i = 0; i < id_list_x_.size(); i++) data_int_list[i] = dynamixel_chain[id_list_x_[i]].goal_position;
    dyn_comm_.SyncWrite(id_list_x_, goal_position, data_int_list);
}

bool DynamixelHandler::SyncReadPosition(){
    vector<int64_t> data_int_list(id_list_x_.size());
    vector<uint8_t> read_id_list(id_list_x_.size());
    for (size_t i = 0; i < id_list_x_.size(); i++) data_int_list[i] = dynamixel_chain[id_list_x_[i]].present_position; // read失敗時に初期化されないままだと危険なので．
    for (size_t i = 0; i < id_list_x_.size(); i++) read_id_list[i]  = 255; // あり得ない値(idは0~252)に設定して，read失敗時に検出できるようにする

    int num_success = (has_hardware_error)
                        ? dyn_comm_.SyncRead     (id_list_x_, present_position, data_int_list, read_id_list)  // エラーがあるときは遅い方
                        : dyn_comm_.SyncRead_fast(id_list_x_, present_position, data_int_list, read_id_list); // エラーがあるときは早い方
    // エラー処理
    if (num_success != id_list_x_.size()){
        ROS_WARN("SyncReadPosition: %d servo(s) failed to read", (int)(id_list_x_.size() - num_success));
        for (size_t i = 0; i < id_list_x_.size(); i++){
            if (std::find(id_list_x_.begin(), id_list_x_.end(), read_id_list[i]) == id_list_x_.end())
                ROS_WARN("  * servo id [%d] failed to read", id_list_x_[i]);
        }
    }
    // 読み込んだデータをdynamixel_chainに反映
    for (size_t i = 0; i < id_list_x_.size(); i++) // data_int_listの初期値がpresent_positionなので，read失敗時はそのままになる．
        dynamixel_chain[id_list_x_[i]].present_position = data_int_list[i]; 

    return num_success>0 ? true : false; // 1つでも成功したら成功とする.あえて冗長に書いている.
}

void DynamixelHandler::SyncReadHardwareError(){
    ROS_INFO("SyncReadHardwareError: Checking hardware error");
    
    vector<int64_t> error_int_list(id_list_x_.size());
    vector<uint8_t> read_id_list(id_list_x_.size());
    for (size_t i = 0; i < id_list_x_.size(); i++) error_int_list[i] = 0;   // read失敗時にエラーだと誤認されないように．
    for (size_t i = 0; i < id_list_x_.size(); i++) read_id_list[i]  = 255; // あり得ない値(idは0~252)に設定して，read失敗時に検出できるようにする

    int num_success = dyn_comm_.SyncRead(id_list_x_, hardware_error_status, error_int_list, read_id_list);

    has_hardware_error = false;
    for (auto error : error_int_list) has_hardware_error += (bool)error; // errorがあるときは0以外の値になる．
    if ( !has_hardware_error ) return;

    ROS_ERROR("SyncReadHardwareError: Hardware Error is detected");
    for (int i = 0; i < id_list_x_.size(); i++) {
        uint8_t error = error_int_list[i];
        if ((error >> HARDWARE_ERROR_INPUT_VOLTAGE)     & 0b1 ) ROS_ERROR("  * servo id [%d] : INPUT_VOLTAGE",     id_list_x_[i]);
        if ((error >> HARDWARE_ERROR_MOTOR_HALL_SENSOR) & 0b1 ) ROS_ERROR("  * servo id [%d] : MOTOR_HALL_SENSOR", id_list_x_[i]);
        if ((error >> HARDWARE_ERROR_OVERHEATING)       & 0b1 ) ROS_ERROR("  * servo id [%d] : OVERHEATING",       id_list_x_[i]);
        if ((error >> HARDWARE_ERROR_MOTOR_ENCODER)     & 0b1 ) ROS_ERROR("  * servo id [%d] : MOTOR_ENCODER",     id_list_x_[i]);
        if ((error >> HARDWARE_ERROR_ELECTRONICAL_SHOCK)& 0b1 ) ROS_ERROR("  * servo id [%d] : ELECTRONICAL_SHOCK",id_list_x_[i]);
        if ((error >> HARDWARE_ERROR_OVERLOAD)          & 0b1 ) ROS_ERROR("  * servo id [%d] : OVERLOAD",          id_list_x_[i]);
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


    // DynamixelのBaudrateを1Mに変更するするときに使った．
    // dyn_comm_.Write(6, baudrate_x, BAUDRATE_INDEX_1M);
    // dyn_comm_.ClosePort();
    // dyn_comm_.set_baudrate(1000000);
    // dyn_comm_.OpenPort();


