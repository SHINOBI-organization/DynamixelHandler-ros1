#include <ros/ros.h>
#include "dynamixel_communicator.h"
#include <vector>
using std::vector;
#include <string>
using std::string;

#include <signal.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamixel_change_baudrate_node");

    ros::NodeHandle nh_p("~");

    int id_max, id_min;
    int baudrate_max, baudrate_min, baudrate_target; 
    int latency_timer;
	string device_name;
    if (!nh_p.getParam("min_id", id_min )) id_min =  0;
    if (!nh_p.getParam("max_id", id_max )) id_max =  50;
    if (!nh_p.getParam("device_name", device_name    )) device_name = "/dev/ttyUSB0";
    if (!nh_p.getParam("min_search_baudrate", baudrate_min)) baudrate_min = 57600;
    if (!nh_p.getParam("max_search_baudrate", baudrate_max)) baudrate_max = 4000000;
    if (!nh_p.getParam("target_baudrate",baudrate_target)) baudrate_target = 1000000;
    if (!nh_p.getParam("latency_timer", latency_timer)) latency_timer = 16;

    auto dyn_comm = DynamixelCommunicator();
    dyn_comm.GetPortHandler(device_name.c_str());
    dyn_comm.set_retry_config(5, 20);

    uint64_t dyn_baudrate;
    switch (baudrate_target) {
        case 9600:    dyn_baudrate = BAUDRATE_INDEX_9600;   break;
        case 57600:   dyn_baudrate = BAUDRATE_INDEX_57600;  break;
        case 115200:  dyn_baudrate = BAUDRATE_INDEX_115200; break;
        case 1000000: dyn_baudrate = BAUDRATE_INDEX_1M;     break;
        case 2000000: dyn_baudrate = BAUDRATE_INDEX_2M;     break;
        case 3000000: dyn_baudrate = BAUDRATE_INDEX_3M;     break;
        case 4000000: dyn_baudrate = BAUDRATE_INDEX_4M;     break;
        case 4500000: dyn_baudrate = BAUDRATE_INDEX_4M5;    break;
        case 6000000: dyn_baudrate = BAUDRATE_INDEX_6M;     break;
        case 10500000:dyn_baudrate = BAUDRATE_INDEX_10M5;   break;
        default: ROS_ERROR("Invalid baudrate %d", baudrate_target); return false;
    }

    vector<int> baudrate_list = {
        9600,
        57600,
        115200,
        1000000,
        2000000,
        3000000,
        4000000,
        4500000,
        6000000,
        10500000
    };

    vector<int> found_ids;
    for ( auto br : baudrate_list ) {
        if ( br < baudrate_min || br > baudrate_max) continue;
        dyn_comm.set_baudrate(br);
        dyn_comm.set_latency_timer( (br<=57600) ? 16 : latency_timer);
        ROS_INFO("Searching Dynamixel baudrate [%d] ...", br);
        if ( !dyn_comm.OpenPort() ) { ROS_ERROR("Failed to open"); continue; }
        ROS_INFO(" ...");
        for (int i=id_min; i<=id_max; i++) {
            if ( std::find(found_ids.begin(), found_ids.end(), i) != found_ids.end() ) continue;
            if ( !dyn_comm.tryPing(i) ) continue;
            ROS_INFO("ID [%d] is found, try change baudrate %d", i, baudrate_target);
            dyn_comm.tryWrite(dyn_x::torque_enable, i, TORQUE_DISABLE);
            dyn_comm.tryWrite(baudrate, i, dyn_baudrate);
            found_ids.push_back(i);
        }
        dyn_comm.ClosePort();
    }

    dyn_comm.set_baudrate(baudrate_target);
    dyn_comm.set_latency_timer( (baudrate_target<=57600) ? 16 : latency_timer);
    if ( !dyn_comm.OpenPort() ) {
        ROS_ERROR("Failed to open USB device [%s]", dyn_comm.port_name().c_str()); 
        return false;
    }

    for ( auto i : found_ids ) {
        if ( dyn_comm.tryPing(i) )
            ROS_INFO("ID [%d] is succeded to change baudrate %d", i, baudrate_target);
        else
            ROS_ERROR("ID [%d] is failed to change baudrate", i);
    }

    ros::shutdown();
}