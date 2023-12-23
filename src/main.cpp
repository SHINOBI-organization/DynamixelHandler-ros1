#include <ros/ros.h>
#include "dynamixel_handler.hpp"

#include <string>
using std::string;

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamixel_handler_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    string DEVICE_NAME;
    if (!nh_p.getParam("DEVICE_NAME",      DEVICE_NAME)) DEVICE_NAME = "/dev/ttyUSB0";
    int         BAUDRATE;
    if (!nh_p.getParam("BAUDRATE",         BAUDRATE)   ) BAUDRATE    =  1000000;
    int         loop_rate;
    if (!nh_p.getParam("loop_rate",        loop_rate)  ) loop_rate   =  50;
    bool        varbose;
    if (!nh_p.getParam("varbose",            varbose)  ) varbose     =  false;
    int         error_ratio;
    if (!nh_p.getParam("error_read_ratio", error_ratio)) error_ratio =  100;
    int id_max;    
    if (!nh_p.getParam("dyn_id_max",   id_max)) id_max = 35;
    assert(0 <= id_max && id_max <= 252);
    
    DynamixelHandler::Initialize(
        DEVICE_NAME,
        BAUDRATE,
        varbose,
        id_max,
        loop_rate,
        error_ratio
    );

    ros::Publisher  pub_dyn_state = nh.advertise<dynamixel_handler::DynamixelState>("/dynamixel/state", 10);

    ros::Rate rate(loop_rate);
    uint cnt = 0;
    while(ros::ok()) {
        // if ( ++cnt % error_ratio == 0 ) { SyncReadHardwareError(); cnt=0; };

        // // Dynamixelから現在角をRead & topicをPublish
        // bool is_success = SyncReadPosition();
        // if ( is_success ) {
        //     dynamixel_handler::DynamixelState msg;
        //     msg.ids.resize(id_list.size());
        //     msg.present_angles.resize(id_list.size());
        //     msg.goal_angles.resize(id_list.size());
        //     for (size_t i = 0; i < id_list.size(); i++) {
        //         msg.ids[i] = id_list[i];
        //         msg.present_angles[i] = pulse2rad(dynamixel_chain[id_list[i]].present_position);
        //         msg.goal_angles[i]    = pulse2rad(dynamixel_chain[id_list[i]].goal_position);
        //     }
        //     pub_dyn_state.publish(msg);
        // }

        // // デバック用
        // if (varbose) ShowDynamixelChain();

        // // topicをSubscribe & Dynamixelへ目標角をWrite
        // ros::spinOnce();
        // rate.sleep();
        // if( is_updated ) {
        //     SyncWritePosition();
        //     is_updated = false;
        // } 
    }
}