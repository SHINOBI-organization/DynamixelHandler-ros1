#ifndef DYNAMIXEL_HANDLER_H_
#define DYNAMIXEL_HANDLER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "dynamixel_communicator.h"
#include <dynamixel_handler/DynamixelCmd.h>
#include <dynamixel_handler/DynamixelCmd_X_ControlPosition.h>
#include <dynamixel_handler/DynamixelCmd_X_ControlVelocity.h>
#include <dynamixel_handler/DynamixelCmd_X_ControlCurrent.h>
#include <dynamixel_handler/DynamixelCmd_X_ControlCurrentPosition.h>
#include <dynamixel_handler/DynamixelCmd_X_ControlExtendedPosition.h>
#include <dynamixel_handler/DynamixelState.h>
#include <dynamixel_handler/DynamixelState_Dynamic.h>

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
#include <utility>
using std::pair;
using std::max;
using std::min;

class DynamixelHandler {
    public:
        //* ROS 初期設定とメインループ
        static bool Initialize();
        static void MainLoop();
        //* ROS subscliber callback関数
        static void BroadcastDynamixelState_Dynamic();
        static void CallBackOfDynamixelCommand(const dynamixel_handler::DynamixelCmd& msg);
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
        static bool SyncReadHardwareError();

        // main loop 内で使う
        static inline bool varbose_      = false;
        static inline int  loop_rate_    = 50;
        static inline int  error_ratio_  = 100;
        // Dynamixelとの通信
        static inline DynamixelComunicator dyn_comm_;
        // 連結しているDynamixelの情報
        static inline vector<uint8_t> id_list_p_; // dynamixel p series
        static inline vector<uint8_t> id_list_x_; // dynamixel x series
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
        static inline map<uint8_t, bool> is_updated_; // cakkbackによって，cmd_valuesが更新されたかどうか．
        static inline map<uint8_t, array<double, 6>> cmd_values_;  // コマンドとして定義した，サーボに書き込む値
        static inline map<uint8_t, array<double, 8>> state_values_;// ステータスとして定義した．サーボから読み取った値
        static inline pair<CmdValues,  CmdValues>   range_write_ = {GOAL_POSITION,    GOAL_PWM};
        static inline pair<StateValues,StateValues> range_read_  = {PRESENT_POSITION, PRESENT_PWM};
        //* cmd_vals と state_vals の更新
        // static void SyncWriteCmdValues();
        static void SyncWriteCmdValues(CmdValues target);
        static void SyncWriteCmdValues(pair<CmdValues, CmdValues> range=range_write_);
        // static bool SyncReadStateValues();
        static bool SyncReadStateValues(StateValues target);
        static bool SyncReadStateValues(pair<StateValues, StateValues> range=range_read_);

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

// ここら変の情報は型番固有の情報なので， dynamixel_parameter.hpp/cpp側に記述して，将来的には自動で読み込ませるようにしたい．
static int64_t deg2pulse(double deg) { return deg * 4096.0 / 360.0 + 2048; }
static double  pulse2deg(int64_t pulse) { return (pulse - 2048 ) * 360.0 / 4096.0; }
static int64_t rad2pulse(double rad) { return rad * 4096.0 / (2.0 * M_PI) + 2048; }
static double  pulse2rad(int64_t pulse) { return (pulse - 2048 ) * 2.0 * M_PI / 4096.0; } // tmp
// int64_t mA2pulse(double mA) { return mA / 1.0; }
// double  pulse2mA(int64_t pulse) { return pulse * 1.0; }


#endif /* DYNAMIXEL_HANDLER_H_ */
