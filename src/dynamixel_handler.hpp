#ifndef DYNAMIXEL_HANDLER_H_
#define DYNAMIXEL_HANDLER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "dynamixel_communicator.h"
#include <dynamixel_handler/DynamixelCmd.h>
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

using namespace dyn_x;

class DynamixelHandler {
    public:
        //* ROS 初期設定とメインループ
        static bool Initialize();
        static void MainLoop();
        //* ROS subscliber callback関数
        static void CallBackOfDynamixelCommand(const dynamixel_handler::DynamixelCmd& msg);
        // static void CallBackOfDynamixelCommand(const dynamixel_handler::DynamixelCmd& msg);　//todo control 方式ごとにコマンドを分ける
        //* ROS publisher publisher instance
        static inline ros::Publisher  pub_dyn_state_;
        // static ros::Publisher  pub_dyn_state_; //todo いくつかのデフォルトを用意しておく
        static inline ros::Subscriber sub_cmd_;
        // static inline ros::Subscriber sub_cmd_; // todo いくつかのデフォルトを用意しておく

    private:
        DynamixelHandler() = delete;
        //* Dynamixelとの通信を超えた機能
        static uint8_t ScanDynamixels(uint8_t id_max);
        //* Dynamixel単体との通信の組み合わせ
        static bool TorqueEnable(uint8_t servo_id);
        static bool TorqueDisable(uint8_t servo_id);
        static bool ClearHardwareError(uint8_t servo_id, DynamixelTorquePermission after_state=TORQUE_ENABLE);
        static bool CheckHardwareError(uint8_t servo_id);
        //* Sync系，Dynamixel複数との通信の組み合わせ
        static void SyncWritePosition();
        static void SyncReadHardwareError();
        static bool SyncReadPosition();
        static void ShowDynamixelChain();
  

        // main loop 内で使う
        static inline bool varbose_      = false;
        static inline int  loop_rate_    = 50;
        static inline int  error_ratio_  = 100;
        // Dynamixelとの通信
        static inline DynamixelComunicator dyn_comm_;
        // 連結しているDynamixelの情報
        static inline vector<uint8_t> id_list_p_; // dynamixel p series
        static inline vector<uint8_t> id_list_x_; // dynamixel x series
        // struct DynamicParamter{
        //     double position = 0;
        //     double velocity = 0;
        //     double current  = 0;
        //     double pwm = 0;
        // }
        // struct StaticParamter{
        //     double torque_constant = 0;
        //     double gear_ratio = 0;
        //     double max_current = 0;
        //     double max_pwm = 0;
        // }
        // struct Dynamixel {
        //     DynamicParamter present;
        //     DynamicParamter goal;
        //     StaticParamter  ;
        // };
        struct Dynamixel {
            int64_t present_position = 0;
            int64_t goal_position    = 0;
        };
        static inline map<uint8_t, Dynamixel> dynamixel_chain; // シリーズにかかわらず，すべてのサーボを格納する
        // その他
        static inline bool has_hardware_error = false;
        static inline bool is_updated = false;
};


// ここら変の情報は型番固有の情報なので， dynamixel_parameter.hpp/cpp側に記述して，将来的には自動で読み込ませるようにしたい．
// int64_t deg2pulse(double deg) { return deg * 4096.0 / 360.0 + 2048; }
// double  pulse2deg(int64_t pulse) { return (pulse - 2048 ) * 360.0 / 4096.0; }
static int64_t rad2pulse(double rad) { return rad * 4096.0 / (2.0 * M_PI) + 2048; }
static double  pulse2rad(int64_t pulse) { return (pulse - 2048 ) * 2.0 * M_PI / 4096.0; } // tmp
// int64_t mA2pulse(double mA) { return mA / 1.0; }
// double  pulse2mA(int64_t pulse) { return pulse * 1.0; }


#endif /* DYNAMIXEL_HANDLER_H_ */
