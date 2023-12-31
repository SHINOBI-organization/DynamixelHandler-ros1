#ifndef DYNAMIXEL_HANDLER_H_
#define DYNAMIXEL_HANDLER_H_

#include <ros/ros.h>
using ros::Time;
#include <std_msgs/String.h>

#include "dynamixel_communicator.h"
#include <dynamixel_handler/DynamixelState.h>
#include <dynamixel_handler/DynamixelStateFree.h>
#include <dynamixel_handler/DynamixelError.h>
#include <dynamixel_handler/DynamixelConfig_Limit.h>
#include <dynamixel_handler/DynamixelConfig_Gain.h>
#include <dynamixel_handler/DynamixelConfig_Mode.h>
#include <dynamixel_handler/DynamixelCommandFree.h>
#include <dynamixel_handler/DynamixelCommand_Profile.h>
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
using std::clamp;

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
        static bool TmpTest();
        static bool Initialize();
        static void MainLoop();
        static void Terminate(int sig);

        //* ROS publishを担う関数と subscliber callback関数
        static void BroadcastDxlState();
        static void BroadcastDxlStateFree(); // todo 
        static void BroadcastDxlError();
        static void BroadcastDxlConfig_Limit(); // todo
        static void BroadcastDxlConfig_Gain();  // todo
        static void BroadcastDxlConfig_Mode();  // todo
        static void CallBackDxlConfig_Limit (const dynamixel_handler::DynamixelConfig_Limit& msg); // todo
        static void CallBackDxlConfig_Gain  (const dynamixel_handler::DynamixelConfig_Gain& msg);  // todo
        static void CallBackDxlConfig_Mode  (const dynamixel_handler::DynamixelConfig_Mode& msg);  // todo
        static void CallBackDxlCommandFree    (const dynamixel_handler::DynamixelCommandFree& msg);    // todo
        static void CallBackDxlCommand_Profile (const dynamixel_handler::DynamixelCommand_Profile& msg); // todo
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
        static inline ros::Subscriber sub_cmd_profile_;
        static inline ros::Subscriber sub_cmd_x_pos_;
        static inline ros::Subscriber sub_cmd_x_vel_;
        static inline ros::Subscriber sub_cmd_x_cur_;
        static inline ros::Subscriber sub_cmd_x_cpos_;
        static inline ros::Subscriber sub_cmd_x_epos_;
        static inline ros::Subscriber sub_config_limit_;
        static inline ros::Subscriber sub_config_gain_;
        static inline ros::Subscriber sub_config_mode_;

    private:
        DynamixelHandler() = delete;
        //* 単体通信を組み合わせた上位機能
        static uint8_t ScanDynamixels(uint8_t id_max);
        static bool ClearHardwareError(uint8_t servo_id, DynamixelTorquePermission after=TORQUE_ENABLE);
        static bool TorqueOn(uint8_t servo_id);
        static bool TorqueOff(uint8_t servo_id);
        static bool StopRotation(uint8_t servo_id);
        static bool ChangeOperatingMode(uint8_t servo_id, DynamixelOperatingMode mode, DynamixelTorquePermission after=TORQUE_ENABLE);
        //* Dynamixel単体との通信による下位機能
        static uint8_t ReadHardwareError(uint8_t servo_id);
        static double  ReadPresentPosition(uint8_t servo_id);
        static double  ReadHomingOffset(uint8_t servo_id);
        static bool    ReadTorqueEnable(uint8_t servo_id);
        static uint8_t ReadOperatingMode(uint8_t servo_id);
        static bool WriteGoalPosition(uint8_t servo_id, double position);
        static bool WriteGoalVelocity(uint8_t servo_id, double velocity);
        static bool WriteHomingOffset(uint8_t servo_id, double offset);
        static bool WriteTorqueEnable(uint8_t servo_id, bool enable);
        static bool WriteOperatingMode(uint8_t servo_id, uint8_t mode);
    
        //* 各種のフラグとパラメータ
        static inline int  loop_rate_    = 50;
        static inline int  ratio_state_pub_  = 1; 
        static inline int  ratio_config_pub_ = 100; // 0の時は初回のみ
        static inline int  ratio_error_pub_  = 100;
        static inline bool use_slipt_write_ = false;
        static inline bool use_slipt_read_  = false;
        static inline bool use_fast_read_   = false;
        static inline int  varbose_mainloop_  = false;
        static inline bool varbose_callback_  = false;
        static inline bool varbose_write_cmd_ = false;
        static inline bool varbose_write_cfg_ = false;
        static inline bool varbose_read_st_      = false;
        static inline bool varbose_read_st_err_  = false;
        static inline bool varbose_read_hwerr_   = false;
        static inline bool varbose_read_cfg_     = false;
        static inline bool varbose_read_cfg_err_ = false;

        //* Dynamixelとの通信
        static inline DynamixelComunicator dyn_comm_;

        //* Dynamixelを扱うための変数群 
        enum CmdValueIndex { //　cmd_values_のIndex, サーボに毎周期で書き込むことができる値
            GOAL_PWM      = 0,
            GOAL_CURRENT  = 1,
            GOAL_VELOCITY = 2,
            PROFILE_ACC   = 3,
            PROFILE_VEL   = 4,
            GOAL_POSITION = 5,
        };
        enum StValueIndex { // state_values_のIndex, サーボから毎周期で読み込むことができる値
            PRESENT_PWM          = 0,
            PRESENT_CURRENT      = 1,
            PRESENT_VELOCITY     = 2,
            PRESENT_POSITION     = 3,
            VELOCITY_TRAJECTORY  = 4,
            POSITION_TRAJECTORY  = 5,
            PRESENT_INPUT_VOLTAGE= 6,
            PRESENT_TEMPERTURE   = 7,
        };
        enum HWErrIndex { // hardware_error_のIndex, サーボが起こしたハードウェアエラー
            INPUT_VOLTAGE      = 0,
            MOTOR_HALL_SENSOR  = 1,
            OVERHEATING        = 2,
            MOTOR_ENCODER      = 3,
            ELECTRONICAL_SHOCK = 4,
            OVERLOAD           = 5,
        };
        enum CfgParamIndex_Limit { // config_param_のIndex, サーボのCmd_valueの制限値
            TEMPERATURE_LIMIT  = 0,
            MAX_VOLTAGE_LIMIT  = 1,
            MIN_VOLTAGE_LIMIT  = 2,
            PWM_LIMIT          = 3,
            CURRENT_LIMIT      = 4,
            ACCELERATION_LIMIT = 5,
            VELOCITY_LIMIT     = 6,
            MAX_POSITION_LIMIT = 7,
            MIN_POSITION_LIMIT = 8,
        };
        // 連結したサーボの基本情報
        static inline vector<uint8_t> id_list_; // chained dynamixel id list
        static inline map<uint8_t, uint16_t> model_; // 各dynamixelの id と model のマップ
        static inline map<uint8_t, uint16_t> series_; // 各dynamixelの id と series のマップ
        // 連結しているサーボの個々の状態を保持するmap
        static inline map<uint8_t, uint8_t> op_mode_; // 各dynamixelの id と 制御モード のマップ
        static inline map<uint8_t, array<double, 6>> cmd_values_;  // 各dynamixelの id と サーボに毎周期で書き込むことができる値のマップ, 中身の並びはCmdValueIndexに対応する
        static inline map<uint8_t, array<double, 8>> state_values_;// 各dynamixelの id と サーボから毎周期で読み込むことができる値のマップ, 中身の並びはStValueIndexに対応する
        static inline map<uint8_t, array<bool,   6>> hardware_error_; // 各dynamixelの id と サーボが起こしたハードウェアエラーのマップ, 中身の並びはHWErrIndexに対応する
        static inline map<uint8_t, array<double,11>> cfg_param_limit_; // 各dynamixelの id と サーボの各種制限値のマップ, 中身の並びはCfgParamIndex_Limitに対応する 
        // 上記の変数を適切に使うための補助的なフラグ
        static inline map<uint8_t, Time> when_op_mode_updated_; // 各dynamixelの id と op_mode_ が更新された時刻のマップ
        static inline map<uint8_t, bool> is_cmd_updated_;      // topicのcallbackによって，cmd_valuesが更新されたかどうかを示すマップ
        static inline bool has_any_hardware_error_    = false; // 連結しているDynamixelのうち，どれか一つでもハードウェアエラーを起こしているかどうか
        static inline bool has_comm_error_read_state_ = false; // 直前のstate_values_の読み込みが通信エラーを起こしたかどうか
        static inline bool is_timeout_read_state_     = false; // 直前のstate_values_の読み込みがタイムアウトしたかどうか
        // 各周期で実行するserial通信の内容を決めるためのset
        static inline set<CmdValueIndex> list_wirte_cmd_  = {};
        static inline set<StValueIndex>  list_read_state_ = {PRESENT_CURRENT, PRESENT_VELOCITY, PRESENT_POSITION};
        //* 連結しているDynamixelに一括で読み書きする関数
        static void SyncWriteCmdValues(CmdValueIndex target);
        static void SyncWriteCmdValues(const set<CmdValueIndex>& list_wirte_cmd=list_wirte_cmd_);
        static void SyncWriteCfgParams_Mode();  // todo 
        static void SyncWriteCfgParams_Gain();  // todo 
        static void SyncWriteCfgParams_Limit(); // todo 
        static bool SyncReadStateValues(StValueIndex target);
        static bool SyncReadStateValues(const set<StValueIndex>& list_read_state=list_read_state_);
        static bool SyncReadHardwareErrors();
        static bool SyncReadCfgParams_Mode();  // todo
        static bool SyncReadCfgParams_Gain();  // todo
        static bool SyncReadCfgParams_Limit(); // todo
};

namespace dyn_x{
const static vector<DynamixelAddress> cmd_dp_list = { // この順序が大事，CmdValueIndexと対応
        goal_pwm            ,
        goal_current        ,
        goal_velocity       ,
        profile_acceleration,
        profile_velocity    ,
        goal_position       
    };
const static vector<DynamixelAddress> state_dp_list = { // この順序が大事，StValueIndexと対応
        present_pwm, 
        present_current, 
        present_velocity, 
        present_position,
        velocity_trajectory,   
        position_trajectory,  
        present_input_voltage, 
        present_temperture    
    }; 
const static vector<DynamixelAddress> cfg_limit_dp_list = { // この順序が大事，CfgParamIndex_Limitと対応
        temperature_limit ,
        max_voltage_limit ,
        min_voltage_limit ,
        pwm_limit         ,
        current_limit     ,
        acceleration_limit,
        velocity_limit    ,
        max_position_limit,
        min_position_limit,
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
const static vector<DynamixelAddress> cfg_limit_dp_list = { // この順序が大事
        temperature_limit ,
        max_voltage_limit ,
        min_voltage_limit ,
        pwm_limit         ,
        current_limit     ,
        acceleration_limit,
        velocity_limit    ,
        max_position_limit,
        min_position_limit,
    };
} // namespace dyn_p

static string control_table_layout(const map<uint8_t, vector<int64_t>>& id_data_map, const vector<DynamixelAddress>& dp_list, const string& header=""){
    std::stringstream ss;
    ss << header << "\n";
    ss << " ID :"; for (const auto& pair : id_data_map) 
                        ss << "  [" << std::setw(3) << (int)pair.first << "] "; ss << "\n";
    for (size_t i = 0; i < dp_list.size(); ++i) {
        ss << "-" << std::setw(4) << dp_list[i].address() ;
        for (const auto& pair : id_data_map)
            ss << std::setw(7) << pair.second[i] << " "; ss << "\n";
    }
    return ss.str();
}

static string id_list_layout(const vector<uint8_t>& id_list, const string& header=""){
    std::stringstream ss;
    ss << header << "\n";
    ss << " ID :"; 
    for ( auto id : id_list ) {
        ss << std::setw(3) << (int)id; 
        if ( id != id_list.back()) ss << ",";
    }
    ss << "\n";
    return ss.str();
}

#endif /* DYNAMIXEL_HANDLER_H_ */
