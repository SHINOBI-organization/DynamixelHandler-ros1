#ifndef DYNAMIXEL_HANDLER_H_
#define DYNAMIXEL_HANDLER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "dynamixel_communicator.h"
#include <dynamixel_handler/DynamixelCmdFree.h>
#include <dynamixel_handler/DynamixelCmd_X_ControlPosition.h>
#include <dynamixel_handler/DynamixelCmd_X_ControlVelocity.h>
#include <dynamixel_handler/DynamixelCmd_X_ControlCurrent.h>
#include <dynamixel_handler/DynamixelCmd_X_ControlCurrentPosition.h>
#include <dynamixel_handler/DynamixelCmd_X_ControlExtendedPosition.h>
#include <dynamixel_handler/DynamixelState.h>

#include <thread>
using std::this_thread::sleep_for;
#include <string>
using std::string;
#include <chrono>
using namespace std::chrono_literals;
#include <map>
using std::map;
#include <vector>
using std::vector;
#include <array>
using std::array;
#include <set>
using std::set;
#include <algorithm>
using std::max_element;
using std::min_element;

static const double DEG = M_PI/180.0; // degを単位に持つ数字に掛けるとradになる
static double deg2rad(double deg){ return deg*DEG; }
static double rad2deg(double rad){ return rad/DEG; }

class DynamixelHandler {
    public:
        //* ROS 初期設定とメインループ
        static bool Initialize();
        static void MainLoop();
        //* ROS subscliber callback関数
        static void BroadcastDynamixelState();
        static void CallBackOfDynamixelCommand(const dynamixel_handler::DynamixelCmdFree& msg);
        static void CallBackOfDxlCmd_X_Position        (const dynamixel_handler::DynamixelCmd_X_ControlPosition& msg);
        static void CallBackOfDxlCmd_X_Velocity        (const dynamixel_handler::DynamixelCmd_X_ControlVelocity& msg);
        static void CallBackOfDxlCmd_X_Current         (const dynamixel_handler::DynamixelCmd_X_ControlCurrent& msg);
        static void CallBackOfDxlCmd_X_CurrentPosition (const dynamixel_handler::DynamixelCmd_X_ControlCurrentPosition& msg);
        static void CallBackOfDxlCmd_X_ExtendedPosition(const dynamixel_handler::DynamixelCmd_X_ControlExtendedPosition& msg);
        //* ROS publisher publisher instance
        static inline ros::Publisher  pub_dyn_state_;
        // static ros::Publisher  pub_dyn_state_; //todo いくつかのデフォルトを用意しておく
        static inline ros::Subscriber cmd_free_;
        static inline ros::Subscriber cmd_x_pos_;
        static inline ros::Subscriber cmd_x_vel_;
        static inline ros::Subscriber cmd_x_cur_;
        static inline ros::Subscriber cmd_x_cpos_;
        static inline ros::Subscriber cmd_x_epos_;

    private:
        DynamixelHandler() = delete;
        //* Dynamixelとの通信を超えた機能
        static uint8_t ScanDynamixels(uint8_t id_max);
        //* Dynamixel単体との通信の組み合わせ
        static bool TorqueEnable(uint8_t servo_id);
        static bool TorqueDisable(uint8_t servo_id);
        static bool ClearHardwareError(uint8_t servo_id, DynamixelTorquePermission after_state=TORQUE_ENABLE);
        static bool CheckHardwareError(uint8_t servo_id);

        // main loop 内で使う
        static inline bool varbose_      = false;
        static inline int  loop_rate_    = 50;
        static inline int  state_pub_ratio_  = 1; 
        static inline int  config_pub_ratio_ = 100; // 0の時は初回のみ
        static inline int  error_pub_ratio_  = 100;
        static inline bool use_slipt_read_   = false;
        // Dynamixelとの通信
        static inline DynamixelComunicator dyn_comm_;
        // 連結しているDynamixelの情報を保持する変数
        enum CmdValues {
            GOAL_PWM             = 0,
            GOAL_CURRENT         = 1,
            GOAL_VOLOCITY        = 2,
            PROFILE_ACCELERATION = 3,
            PROFILE_VELOCITY     = 4,
            GOAL_POSITION        = 5,
        };
        enum StateValues {
            PRESENT_PWM          = 0,
            PRESENT_CURRENT      = 1,
            PRESENT_VELOCITY     = 2,
            PRESENT_POSITION     = 3,
            VELOCITY_TRAJECTORY  = 4,
            POSITION_TRAJECTORY  = 5,
            PRESENT_INPUT_VOLTAGE= 6,
            PRESENT_TEMPERTURE   = 7,
        };
        static inline vector<uint8_t> id_list_; // chained dynamixel id list
        static inline map<uint8_t, uint16_t> model_; // 各dynamixelの id と model のマップ
        static inline map<uint8_t, uint16_t> series_; // 各dynamixelの id と series のマップ
        static inline map<uint8_t, array<double, 6>> cmd_values_;  // 各dynamixelの id と コマンドとして書き込む値 のマップ
        static inline map<uint8_t, array<double, 8>> state_values_;// 各dynamixelの id と 状態として常時読み込む値 のマップ
        static inline map<uint8_t, bool> is_updated_; // topicのcallbackによって，cmd_valuesが更新されたかどうかを示すマップ
        static inline set<CmdValues>   list_wirte_cmd_  = {};
        static inline set<StateValues> list_read_state_ = {PRESENT_POSITION, PRESENT_VELOCITY};
        //* 連結しているDynamixelに一括で読み書きする関数
        static void SyncWriteCmdValues(CmdValues target);
        static void SyncWriteCmdValues(set<CmdValues>& list_wirte_cmd=list_wirte_cmd_);
        static bool SyncReadStateValues(StateValues target);
        static bool SyncReadStateValues(set<StateValues>& list_read_state=list_read_state_);
        static bool SyncReadHardwareError();

        // その他
        static inline bool has_hardware_error = false;
};

namespace dyn_x{
const static vector<DynamixelAddress> cmd_dp_list = { // この順序が大事
        goal_pwm            ,
        goal_current        ,
        goal_velocity       ,
        profile_acceleration,
        profile_velocity    ,
        goal_position       
    };
const static vector<DynamixelAddress> state_dp_list = { // この順序が大事
        present_pwm, 
        present_current, 
        present_velocity, 
        present_position,
        velocity_trajectory,   
        position_trajectory,  
        present_input_voltage, 
        present_temperture    
    }; 
} // namespace dyn_x

namespace dyn_p{
const static vector<DynamixelAddress> cmd_dp_list = { // この順序が大事
        goal_pwm,
        goal_current,
        goal_velocity,
        profile_acceleration,
        profile_velocity,
        goal_position
    };
const static vector<DynamixelAddress> state_dp_list = { // この順序が大事
        present_pwm, 
        present_current, 
        present_velocity, 
        present_position,
        velocity_trajectory,   
        position_trajectory,  
        present_input_voltage, 
        present_temperture    
    }; 
} // namespace dyn_p

#endif /* DYNAMIXEL_HANDLER_H_ */
