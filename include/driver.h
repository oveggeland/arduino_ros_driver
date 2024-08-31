#pragma once

#include "ros/ros.h"

#include <bits/stdc++.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <fcntl.h>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

#include "arduino/arduino_status_msg.h"
#include "arduino/arduino_config_msg.h"
#include "arduino/arduino_config_srv.h"

#include "protocol.h"

#define BUFFER_SIZE 1472
#define PORT 5005
#define CONNECTION_TIMEOUT ros::Duration(2)

class ArduinoDriver {
    public:
        ArduinoDriver(ros::NodeHandle nh);
        ~ArduinoDriver();

        void checkSocket();
        void checkTimeout();
    private:
        static bool configService(arduino::arduino_config_srv::Request &req, arduino::arduino_config_srv::Response &res);

        void parseBuffer(int buffer_size);
        void setupSocket();
        void setupConfig();

        void imuData(imuPackage* p_pkg);
        void gnssData(gnssPackage* p_pkg);
        void statusUpdate(arduinoStatus* p_pkg);

        char imu_header[4] = IMU_HEADER;
        char gnss_header[5] = GNSS_HEADER;
        char status_header[3] = STATUS_HEADER;

        ros::Publisher imu_pub;
        ros::Publisher gnss_pub;
        ros::Publisher status_pub;

        ros::ServiceServer config_service;

        ros::Time t_last_contact;

        // Socket stuff
        int sockfd;
        char buffer[BUFFER_SIZE]; 
};