#ifndef UDP_SEND_UDP_HPP
#define UDP_SEND_UDP_HPP

#endif //UDP_SEND_UDP_HPP

#include <ros/ros.h>
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <arpa/inet.h>
#include <queue>
#include <chrono>
#include <sensor_msgs/Joy.h>
#include "std_msgs/Float32.h"

using namespace std;
namespace carnet {
class udp_car {
public:
    udp_car(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
            std::string node_name);

    // ros
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Timer loop_timer_;
    std::string node_name_{"udp_sender_node"};
    ros::Subscriber joy_sub_;

    std_msgs::Float32 cmd_throttle;
    std_msgs::Float32 cmd_brake;
    std_msgs::Float32 cmd_steer;

    bool control_enable, park_enable, reverse_enable, neutral_enable, drive_enable;
    int driver_control_enable, driver_control_enable_cancel, gear_neutral, gear_park, gear_drive, gear_reverse;
    int driver_control_position, driver_control_cancel_position, steer_position, throttle_position, brake_position, gear_neutral_position, gear_park_position, gear_drive_position, gear_reserve_position;

    //// UDP Settings
    /* local address and port */
    int CarNetSocket;
    sockaddr_in addr_local;
    int local_port;

    /* remote address and port */
    sockaddr_in addr_remote;
    socklen_t addr_remote_len;
    std::string remote_ip;
    int remote_port;

    void initialize();
    void joyCallback(const sensor_msgs::Joy joy);
    void timerCb();
    bool initSocket(void);
    void sendmsgs();
};
}//namespace carnet
