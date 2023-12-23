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

using namespace dyn_x; // 一旦すべてのサーボをx_seriesとして扱う

class DynamixelHandler {
    public:
        static void Initialize(string device_name, int baudrate, bool varbose, int id_max ,int loop_rate, int error_ratio) {   
            dyn_comm_ = DynamixelComunicator(device_name.c_str(), baudrate);
            varbose_     = varbose;
            loop_rate_   = loop_rate;
            error_ratio_ = error_ratio;
            is_updated   = false;
            has_hardware_error = false;

            ros::NodeHandle nh;
            sub_cmd_ = nh.subscribe("/dynamixel/cmd",   10, DynamixelHandler::CallBackOfDynamixelCommand);

            // 通信の開始
            dyn_comm_.OpenPort();

            // id_listの作成
            ROS_INFO("Auto scanning Dynamixel (id range 1 to [%d])", id_max);
            bool any_found = ScanDynamixels(id_max);
            if( !any_found ) ROS_ERROR("Dynamicel is not found in USB device [%s]", dyn_comm_.port_name().c_str());

            // プログラム内部の変数であるdynamixel_chainの初期化
            dynamixel_chain.resize(id_max);
            std::fill(dynamixel_chain.begin(), dynamixel_chain.end(), Dynamixel{0,0});
            for (auto id : id_list_x_) {
                dynamixel_chain[id].present_position = dyn_comm_.Read(present_position, id); // エラー時は0
                dynamixel_chain[id].goal_position    = dyn_comm_.Read(goal_position, id);    // エラー時は0
            }

            // サーボの実体としてのDynamixel Chainの初期化, 今回は一旦すべて電流制御付き位置制御モードにしてトルクON    
            for (auto id : id_list_x_) {
                if ( TorqueDisable(id) ) ROS_WARN("Servo id [%d] failed to disable torque", id);
                dyn_comm_.Write(torque_enable, id, TORQUE_DISABLE);
                dyn_comm_.Write(operating_mode, id, OPERATING_MODE_EXTENDED_POSITION);  
                dyn_comm_.Write(profile_acceleration, id, 500); // 0~32767 数字は適当
                dyn_comm_.Write(profile_velocity, id, 100); // 0~32767 数字は適当
                int present_pos = dyn_comm_.Read(present_position, id);
                dyn_comm_.Write(goal_position, id, present_pos);
                bool is_hardware_error = dyn_comm_.Read(hardware_error_status, id); // ここでは雑に判定している．本来の返り値はuint8_tで各ビットに意味がある. 
                if (is_hardware_error) ClearError(id, TORQUE_DISABLE);
                if ( TorqueEnable(id) ) ROS_WARN("Servo id [%d] failed to enable torque", id);
            }
        }

        // callback関数
        static void CallBackOfDynamixelCommand(const dynamixel_handler::DynamixelCmd& msg);
        // Dynamixelとの通信を超えた機能
        static bool ScanDynamixels(uint8_t id_max);
        // Dynamixel単体との通信の組み合わせ
        static bool ClearError(uint8_t servo_id, DynamixelTorquePermission after_state=TORQUE_ENABLE);
        static bool TorqueEnable(uint8_t servo_id);
        static bool TorqueDisable(uint8_t servo_id);
        // Sync系，Dynamixel複数との通信の組み合わせ
        static void SyncWritePosition();
        static void SyncReadHardwareError();
        static bool SyncReadPosition();
        static void ShowDynamixelChain();
  
        struct Dynamixel{
            int32_t goal_position;
            int32_t present_position;
        };
        // ROS関係
        static ros::Subscriber sub_cmd_;
        // main loop 内で使う // ROS1の実装ではほぼ使わない
        static bool        varbose_;
        static int         loop_rate_;
        static int         error_ratio_;
        // Dynamixelとの通信
        static DynamixelComunicator dyn_comm_;
        // 連結しているDynamixelの情報
        static vector<uint8_t> id_list_p_; // dynamixel pro series
        static vector<uint8_t> id_list_x_; // dynamixel x series
        static vector<Dynamixel> dynamixel_chain;
        // その他フラグ系
        static bool is_updated;
        static bool has_hardware_error;
};


#endif /* DYNAMIXEL_HANDLER_H_ */
