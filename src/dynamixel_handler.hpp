#ifndef DYNAMIXEL_HANDLER_H_
#define DYNAMIXEL_HANDLER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "dynamixel_communicator.h"
#include <dynamixel_handler/DynamixelState.h>
#include <dynamixel_handler/DynamixelStateFree.h>
#include <dynamixel_handler/DynamixelError.h>
#include <dynamixel_handler/DynamixelConfig_Limit.h>
#include <dynamixel_handler/DynamixelConfig_Gain.h>
#include <dynamixel_handler/DynamixelConfig_Mode.h>
#include <dynamixel_handler/DynamixelCommandFree.h>
#include <dynamixel_handler/DynamixelCommand_Option.h>
#include <dynamixel_handler/DynamixelCommand_X_ControlPosition.h>
#include <dynamixel_handler/DynamixelCommand_X_ControlVelocity.h>
#include <dynamixel_handler/DynamixelCommand_X_ControlCurrent.h>
#include <dynamixel_handler/DynamixelCommand_X_ControlCurrentPosition.h>
#include <dynamixel_handler/DynamixelCommand_X_ControlExtendedPosition.h>

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

/**
 * DynamixelをROSで動かすためのクラス．本pkgのメインクラス． 
 * 基本的に Initialize() 呼出し後， while(ros::ok())内で MainLoop() を呼び出すだけでよい．
 * その他の関数は Initialize() と MainLoop() のロジックを構成するための部品に過ぎない．
 * 
 * 検出したDynamixelに関して，idのみベクトルとして保持し，
 * それ以外の情報はすべて，idをキーとしたmapに保持している．
*/
class DynamixelHandler {
    public:
        //* ROS 初期設定とメインループ
        static bool Initialize();
        static void MainLoop();
        //* ROS publishを担う関数と subscliber callback関数
        static void BroadcastDxlState();
        static void BroadcastDxlStateFree();
        static void BroadcastDxlError();
        static void BroadcastDxlConfig_Limit();
        static void BroadcastDxlConfig_Gain();
        static void BroadcastDxlConfig_Mode();
        static void CallBackDxlConfig_Limit (const dynamixel_handler::DynamixelConfig_Limit& msg);
        static void CallBackDxlConfig_Gain  (const dynamixel_handler::DynamixelConfig_Gain& msg);
        static void CallBackDxlConfig_Mode  (const dynamixel_handler::DynamixelConfig_Mode& msg);
        static void CallBackDxlCommandFree    (const dynamixel_handler::DynamixelCommandFree& msg);
        static void CallBackDxlCommand_Option (const dynamixel_handler::DynamixelCommand_Option& msg);
        static void CallBackDxlCommand_X_Position        (const dynamixel_handler::DynamixelCommand_X_ControlPosition& msg);
        static void CallBackDxlCommand_X_Velocity        (const dynamixel_handler::DynamixelCommand_X_ControlVelocity& msg);
        static void CallBackDxlCommand_X_Current         (const dynamixel_handler::DynamixelCommand_X_ControlCurrent& msg);
        static void CallBackDxlCommand_X_CurrentPosition (const dynamixel_handler::DynamixelCommand_X_ControlCurrentPosition& msg);
        static void CallBackDxlCommand_X_ExtendedPosition(const dynamixel_handler::DynamixelCommand_X_ControlExtendedPosition& msg);
        //* ROS publisher subscriber instance
        static inline ros::Publisher  pub_state_;
        static inline ros::Publisher  pub_state_free_;
        static inline ros::Publisher  pub_error_;
        static inline ros::Publisher  pub_config_limit_;
        static inline ros::Publisher  pub_config_gain_;
        static inline ros::Publisher  pub_config_mode_;
        static inline ros::Subscriber sub_cmd_free_;
        static inline ros::Subscriber sub_cmd_option_;
        static inline ros::Subscriber sub_cmd_x_pos_;
        static inline ros::Subscriber sub_cmd_x_vel_;
        static inline ros::Subscriber sub_cmd_x_cur_;
        static inline ros::Subscriber sub_cmd_x_cpos_;
        static inline ros::Subscriber sub_cmd_x_epos_;

    private:
        DynamixelHandler() = delete;
        //* Dynamixelとの通信を超えた機能
        static uint8_t ScanDynamixels(uint8_t id_max);
        //* Dynamixel単体との通信の組み合わせ
        static bool TorqueEnable(uint8_t servo_id);
        static bool TorqueDisable(uint8_t servo_id);
        static bool ClearHardwareError(uint8_t servo_id, DynamixelTorquePermission after_state=TORQUE_ENABLE);
        static bool CheckHardwareError(uint8_t servo_id);

        //* 各種のフラグとパラメータ
        static inline int  loop_rate_    = 50;
        static inline int  state_pub_ratio_  = 1; 
        static inline int  config_pub_ratio_ = 100; // 0の時は初回のみ
        static inline int  error_pub_ratio_  = 100;
        static inline bool use_slipt_read_  = false;
        static inline bool use_fast_read_   = false;
        static inline int  varbose_mainloop_  = false;
        static inline bool varbose_callback_  = false;
        static inline bool varbose_write_cmd_ = false;
        static inline bool varbose_write_cfg_ = false;
        static inline bool varbose_read_st_     = false;
        static inline bool varbose_read_st_err_ = false;
        static inline bool varbose_read_hwerr_  = false;
        static inline bool varbose_read_cfg_    = false;
        //* Dynamixelとの通信
        static inline DynamixelComunicator dyn_comm_;
        //* Dynamixelを扱うための変数群 
        enum CmdValues { //　cmd_values_のIndex, サーボに毎周期で書き込むことができる値
            GOAL_PWM             = 0,
            GOAL_CURRENT         = 1,
            GOAL_VELOCITY        = 2,
            PROFILE_ACCELERATION = 3,
            PROFILE_VELOCITY     = 4,
            GOAL_POSITION        = 5,
        };
        enum StateValues { // state_values_のIndex, サーボから毎周期で読み込むことができる値
            PRESENT_PWM          = 0,
            PRESENT_CURRENT      = 1,
            PRESENT_VELOCITY     = 2,
            PRESENT_POSITION     = 3,
            VELOCITY_TRAJECTORY  = 4,
            POSITION_TRAJECTORY  = 5,
            PRESENT_INPUT_VOLTAGE= 6,
            PRESENT_TEMPERTURE   = 7,
        };
        enum HardwareErrors { // hardware_error_のIndex, サーボが起こしたハードウェアエラー
            INPUT_VOLTAGE      = 0, // HARDWARE_ERROR_INPUT_VOLTAGE     
            MOTOR_HALL_SENSOR  = 1, // HARDWARE_ERROR_MOTOR_HALL_SENSOR 
            OVERHEATING        = 2, // HARDWARE_ERROR_OVERHEATING       
            MOTOR_ENCODER      = 3, // HARDWARE_ERROR_MOTOR_ENCODER     
            ELECTRONICAL_SHOCK = 4, // HARDWARE_ERROR_ELECTRONICAL_SHOCK
            OVERLOAD           = 5, // HARDWARE_ERROR_OVERLOAD          
        };
        // 連結したサーボの基本情報
        static inline vector<uint8_t> id_list_; // chained dynamixel id list
        static inline map<uint8_t, uint16_t> model_; // 各dynamixelの id と model のマップ
        static inline map<uint8_t, uint16_t> series_; // 各dynamixelの id と series のマップ
        // 連結しているサーボの個々の状態を保持するmap
        static inline map<uint8_t, array<double, 6>> cmd_values_;  // 各dynamixelの id と サーボに毎周期で書き込むことができる値のマップ, 中身とIndexははCmdValuesに対応する
        static inline map<uint8_t, array<double, 8>> state_values_;// 各dynamixelの id と サーボから毎周期で読み込むことができる値のマップ, 中身とIndexははStateValuesに対応する
        static inline map<uint8_t, array<bool,   6>> hardware_error_; // 各dynamixelの id と サーボが起こしたハードウェアエラーのマップ, 中身とIndexははHardwareErrorsに対応する
        // 上記の変数を適切に使うための補助的なフラグ
        static inline map<uint8_t, bool> is_cmd_updated_; // topicのcallbackによって，cmd_valuesが更新されたかどうかを示すマップ
        static inline bool has_any_hardware_error_ = false; // 連結しているDynamixelのうち，どれか一つでもハードウェアエラーを起こしているかどうか
        static inline bool was_timeout_read_state_ = false; // 直前のstate_values_の読み込みがタイムアウトしたかどうか
        // 各周期で実行するserial通信の内容を決めるためのset
        static inline set<CmdValues>   list_wirte_cmd_  = {};
        static inline set<StateValues> list_read_state_ = {PRESENT_CURRENT, PRESENT_VELOCITY, PRESENT_POSITION};
        //* 連結しているDynamixelに一括で読み書きする関数
        static void SyncWriteCmdValues(CmdValues target);
        static void SyncWriteCmdValues(set<CmdValues>& list_wirte_cmd=list_wirte_cmd_);
        static bool SyncReadStateValues(StateValues target);
        static bool SyncReadStateValues(set<StateValues>& list_read_state=list_read_state_);
        static bool SyncReadHardwareError();


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
