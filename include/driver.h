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

#include "protocol.h"

#define BUFFER_SIZE 1472
#define PORT 5005

class ArduinoDriver {
    public:
        ArduinoDriver(ros::NodeHandle nh);
        ~ArduinoDriver();

        void checkSocket();
    private:
        void parseBuffer(int buffer_size);
        void setupSocket();
        void setupConfig();

        void imuData(imuPackage* p_pkg);
        void gnssData(gnssPackage* p_pkg);
        void status(arduinoStatus* p_pkg);

        char imu_header[4] = IMU_HEADER;
        char gnss_header[5] = GNSS_HEADER;
        char status_header[3] = STATUS_HEADER;

        ros::Publisher imu_pub;
        ros::Publisher gnss_pub;
        ros::Publisher status_pub;
        ros::ServiceServer config_service;

        // Socket stuff
        int sockfd;
        char buffer[BUFFER_SIZE]; 
};