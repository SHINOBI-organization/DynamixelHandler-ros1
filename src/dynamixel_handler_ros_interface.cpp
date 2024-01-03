#include "dynamixel_handler.hpp"

using namespace dyn_x;

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
