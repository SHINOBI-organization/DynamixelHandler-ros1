#include <ros/ros.h>
#include "dynamixel_communicator.h"

#include <signal.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamixel_change_baudrate_node");

    ros::NodeHandle nh_p("~");

    int id_max, id_min;
    int baudrate_prev, baudrate_after; 
	 string device_name;
    if (!nh_p.getParam("min_id", id_min )) id_min =  0;
    if (!nh_p.getParam("max_id", id_max )) id_max =  50;
    if (!nh_p.getParam("device_name",      device_name   )) device_name = "/dev/ttyUSB0";
    if (!nh_p.getParam("prev_baudrate",    baudrate_prev )) baudrate_prev  = 57600;
    if (!nh_p.getParam("after_baudrate",   baudrate_after)) baudrate_after = 1000000;
    auto dyn_comm = DynamixelCommunicator(device_name.c_str(), baudrate_prev, 16);
    if ( !dyn_comm.OpenPort() ) {
        ROS_ERROR("Failed to open USB device [%s]", dyn_comm.port_name().c_str()); 
        return false;
    }
    dyn_comm.set_retry_config(10, 5);

    uint64_t dyn_baudrate;
    switch (baudrate_after) {
        case 9600:    dyn_baudrate = BAUDRATE_INDEX_9600;   break;
        case 57600:   dyn_baudrate = BAUDRATE_INDEX_57600;  break;
        case 115200:  dyn_baudrate = BAUDRATE_INDEX_115200; break;
        case 1000000: dyn_baudrate = BAUDRATE_INDEX_1M;     break;
        case 2000000: dyn_baudrate = BAUDRATE_INDEX_2M;     break;
        case 3000000: dyn_baudrate = BAUDRATE_INDEX_3M;     break;
        case 4000000: dyn_baudrate = BAUDRATE_INDEX_4M;     break;
        case 4500000: dyn_baudrate = BAUDRATE_INDEX_4M5;    break;
        case 10500000:dyn_baudrate = BAUDRATE_INDEX_10M5;   break;
        default: ROS_ERROR("Invalid baudrate %d", baudrate_after); return false;
    }

    ROS_INFO("Searching Dynamixel servos...");
    vector<int> found_ids;
    for (int i=id_min; i<=id_max; i++) {
        if ( !dyn_comm.tryPing(i) ) continue;
        ROS_INFO("Servo ID %d is found, try change baudrate %d", i, baudrate_after);
        dyn_comm.tryWrite(baudrate, i, dyn_baudrate);
        found_ids.push_back(i);
    }

    if ( found_ids.empty() ) {
        ROS_ERROR("No Dynamixel servo is found");
        return false;
    }

    dyn_comm.ClosePort();
    dyn_comm.set_baudrate(baudrate_after);
    if ( !dyn_comm.OpenPort() ) {
        ROS_ERROR("Failed to open USB device [%s]", dyn_comm.port_name().c_str()); 
        return false;
    }

    for ( auto i : found_ids ) {
        if ( dyn_comm.tryPing(i) )
            ROS_INFO("Servo ID %d is succeded to change baudrate %d", i, baudrate_after);
        else
            ROS_ERROR("Servo ID %d is failed to change baudrate", i);
    }

    ros::shutdown();
}