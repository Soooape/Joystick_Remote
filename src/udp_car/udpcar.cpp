#include "udpcar.hpp"

namespace carnet {
    udp_car::udp_car(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                     std::string node_name)
            : nh_(nh),
              control_enable(false),
              park_enable(true),reverse_enable(false),neutral_enable(false),drive_enable(false),
              nh_private_(nh_private),
              node_name_(node_name) {
        initialize();
    }

    void udp_car::joyCallback(const sensor_msgs::Joy joy) {
        cmd_steer.data = 300 * joy.axes[steer_position];//方向盘转角
        cmd_brake.data = -50 * (joy.axes[brake_position]-1);//制动踏板行程
        cmd_throttle.data = -50 * (joy.axes[throttle_position]-1);//油门踏板行程
        driver_control_enable = joy.buttons[driver_control_position];
        driver_control_enable_cancel = joy.buttons[driver_control_cancel_position];
        gear_neutral = joy.buttons[gear_neutral_position];
        gear_park = joy.buttons[gear_park_position];
        gear_drive = joy.buttons[gear_drive_position];
        gear_reverse = joy.buttons[gear_reserve_position];
    }

    bool udp_car::initSocket() {

        this->addr_remote_len = sizeof(this->addr_local);

        //// create a socket
        this->CarNetSocket = socket(AF_INET, SOCK_DGRAM, 0);
        if (this->CarNetSocket < 0) {
            perror("create CarNetSocket failed!\n");
            return false;
        } else {
            std::cout << "create CarNetSocket succeed!" << std::endl;
        }
        //// set the local address
        memset((char *) &addr_local, 0, sizeof(addr_local));
        this->addr_local.sin_addr.s_addr = htonl(INADDR_ANY);
        this->addr_local.sin_family = AF_INET;
        this->addr_local.sin_port = htons(local_port);

        //// bind the socket with local address
        if (bind(CarNetSocket, (sockaddr *) &addr_local, sizeof(sockaddr)) < 0) {
            perror("bind the CarNetSocket failed!");
            return false;
        } else {
            std::cout << "bind the CarNetSocket succeed!" << std::endl;
            std::cout << "Local Port : " << this->local_port << std::endl;
        }
        //// set the remote address
        memset(&addr_remote, 0, sizeof(addr_remote));
        this->addr_remote.sin_addr.s_addr = inet_addr(remote_ip.c_str());
        this->addr_remote.sin_family = AF_INET;
        this->addr_remote.sin_port = htons(remote_port);

        std::cout << "Remote IP  : " << this->remote_ip.c_str() << std::endl;
        std::cout << "Remote Port: " << this->remote_port << std::endl;

        return true;
    }

    void udp_car::sendmsgs() {
        int steer;
            steer = cmd_steer.data;
        int throttle = cmd_throttle.data;
        int brake = cmd_brake.data;
        if (!this->control_enable){
            steer = 0;
            throttle = 0;
            brake = 0;
            std::cout << "控制未使能" << std::endl;}
        else {
            std::cout << "控制使能" << std::endl;
        }
        if(gear_park){
            park_enable = true, reverse_enable = false, neutral_enable = false, drive_enable = false;}
        if(gear_reverse){
            park_enable = false, reverse_enable = true, neutral_enable = false, drive_enable = false;}
        if(gear_neutral){
            park_enable = false, reverse_enable = false, neutral_enable = true, drive_enable = false;}
        if(gear_drive){
            park_enable = false, reverse_enable = false, neutral_enable = false, drive_enable = true;}

        unsigned char can110[13];
        memset(can110, 0, sizeof(can110));
        can110[0] = 0x02;
        can110[1] = 0x00;
        can110[2] = 0x00;
        can110[3] = 0x01;
        can110[4] = 0x10;
        if(control_enable)
            can110[5] = 0x01;
        if(!control_enable)
            can110[5] = 0x00;
        can110[6] = throttle;

        unsigned char can111[13];
        memset(can111, 0, sizeof(can111));
        can111[0] = 0x02;
        can111[1] = 0x00;
        can111[2] = 0x00;
        can111[3] = 0x01;
        can111[4] = 0x11;
        if(control_enable)
            can111[5] = 0x01;
        if(!control_enable)
            can111[5] = 0x00;
        can111[6] = brake;

        unsigned char can112[13];
        memset(can112, 0, sizeof(can112));
        can112[0] = 0x03;
        can112[1] = 0x00;
        can112[2] = 0x00;
        can112[3] = 0x01;
        can112[4] = 0x12;
        if(control_enable)
            can112[5] = 0x01;
        if(!control_enable)
            can112[5] = 0x00;
        can112[6] = steer;
        can112[7] = (steer & 0xff00)>>8;

        unsigned char can114[13];
        memset(can114, 0, sizeof(can114));
        can114[0] = 0x01;
        can114[1] = 0x00;
        can114[2] = 0x00;
        can114[3] = 0x01;
        can114[4] = 0x14;
        if(park_enable)
            can114[5] = 0x01;
        if(reverse_enable)
            can114[5] = 0x02;
        if(neutral_enable)
            can114[5] = 0x03;
        if(drive_enable)
            can114[5] = 0x04;

        cout << "CAN110: ";
        for (int i = 3; i < 13; i++) {
            printf(" %X", can110[i]);
        }
        cout << endl;
        
        cout << "CAN111: ";
        for (int i = 3; i < 13; i++) {
            printf(" %X", can111[i]);
        }
        cout << endl;

        cout << "CAN112: ";
        for (int i = 3; i < 13; i++) {
            printf(" %X", can112[i]);
        }
        cout << endl;

        cout << "CAN114: ";
        for (int i = 3; i < 13; i++) {
            printf(" %X", can114[i]);
        }
        cout << endl;

        sendto(this->CarNetSocket, can110, sizeof(can110), 0, (struct sockaddr *) &addr_remote,
                   addr_remote_len);
        sendto(this->CarNetSocket, can111, sizeof(can111), 0, (struct sockaddr *) &addr_remote,
                   addr_remote_len);
        sendto(this->CarNetSocket, can112, sizeof(can112), 0, (struct sockaddr *) &addr_remote,
               addr_remote_len);
        sendto(this->CarNetSocket, can114, sizeof(can114), 0, (struct sockaddr *) &addr_remote,
               addr_remote_len);
    }

    void udp_car::initialize() {
        local_port = 8500;
        remote_ip = "192.168.0.10";
        remote_port = 8400;

        nh_.param("axis_steer", steer_position, steer_position);
        nh_.param("axis_brake", brake_position, brake_position);
        nh_.param("axis_throttle", throttle_position, throttle_position);
        nh_.param("driver_control_position", driver_control_position, driver_control_position);
        nh_.param("driver_control_cancel_position", driver_control_cancel_position, driver_control_cancel_position);
        nh_.param("gear_neutral_position", gear_neutral_position, gear_neutral_position);
        nh_.param("gear_park_position", gear_park_position, gear_park_position);
        nh_.param("gear_drive_position", gear_drive_position, gear_drive_position);
        nh_.param("gear_reserve_position", gear_reserve_position, gear_reserve_position);


        initSocket();
        joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &udp_car::joyCallback, this);
        loop_timer_ = nh_.createTimer(ros::Duration(0.05), boost::bind(&udp_car::timerCb, this));
    }

    void udp_car::timerCb() {
        if (driver_control_enable) {
            control_enable = true;
        } else if (driver_control_enable_cancel) {
            control_enable = false;
        }
        sendmsgs();
    }
//namespace carnet
}
    int main(int argc, char **argv) {
        std::string node_name = "udp_car";
        ros::init(argc, argv, node_name);
        ros::NodeHandle nh("");
        ros::NodeHandle nh_private("~");
        carnet::udp_car sender(nh, nh_private, node_name);
        ROS_INFO("Initialized sender node.");
        ros::spin();
    }
