#include "ros/ros.h"
#include <ros/package.h>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

#include "control.h"
#include "protocol.h"

#include <iostream>
#include <sstream>
#include <csignal>

#include <bits/stdc++.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <fcntl.h>

#define BUFFER_SIZE 1472
#define PORT 5005

using namespace std;

inline void SignalHandler(int signum) {
  ros::shutdown();
  exit(signum);
}


class ArduinoDriver {
    public:
        ArduinoDriver(ros::NodeHandle nh);
        ~ArduinoDriver();

        void checkSocket();
    private:
        void parseBuffer(int buffer_size);
        void setupSocket();

        char imu_header[4] = IMU_HEADER;

        ros::Publisher imu_pub;
        ros::Publisher gnss_pub;
        ros::Publisher status_pub;
        ros::ServiceServer config_service;

        // Socket stuff
        int sockfd;
        char buffer[BUFFER_SIZE]; 
};

void ArduinoDriver::parseBuffer(int buffer_size){
    ROS_INFO("Received a buffer");
}

void ArduinoDriver::checkSocket(){    
    int bytes_received = recv(sockfd, buffer, BUFFER_SIZE, 0); 
    if (recv(sockfd, buffer, BUFFER_SIZE, 0) > 0){
        parseBuffer(bytes_received);
    }
}

void ArduinoDriver::setupSocket(){
    // Create socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0); 
    if ( sockfd < 0 ) { 
        ROS_FATAL("Arduino driver: Could not create socket");
        exit(EXIT_FAILURE); 
    } 
    
    // Allow reuse of port+address
    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET,
                SO_REUSEADDR | SO_REUSEPORT, &opt,
                sizeof(opt));

    // Make socket non-blocking
    fcntl(sockfd, F_SETFL, O_NONBLOCK);

    // Bind to socket
    struct sockaddr_in servaddr; 
    memset(&servaddr, 0, sizeof(servaddr)); 
    servaddr.sin_family    = AF_INET; // IPv4 
    servaddr.sin_addr.s_addr = INADDR_ANY; 
    servaddr.sin_port = htons(PORT); 

    if ( bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 ) 
    { 
        ROS_FATAL("Arduino driver: Could not bind socket");
        exit(EXIT_FAILURE); 
    } 
}

ArduinoDriver::ArduinoDriver(ros::NodeHandle nh){
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);
    ros::Publisher gnss_pub = nh.advertise<sensor_msgs::NavSatFix>("gnss", 10);
    ros::Publisher status_pub = nh.advertise<arduino::arduino_status_msg>("status", 5);

    setupSocket();
}

ArduinoDriver::~ArduinoDriver(){
    config_service.shutdown();
}

void sendImuMsg(imuPackage* p_imu_pkg){
    ROS_INFO_STREAM(p_imu_pkg->acc[2]);
}

void parseBuffer(char* buffer, int bytesReceived){
    char* p_end = buffer + bytesReceived;
    char* p_frame = buffer;

    char imu_header[4] = {'$', 'I', 'M', 'U'};

    while (p_frame < p_end){
        p_frame = (char*) memchr(p_frame, '$', p_end - p_frame);

        if (p_frame == nullptr){
            break;
        }

        else if (memcmp(p_frame, imu_header, 4) == 0 && (p_end - p_frame) > sizeof(imuPackage)){
            sendImuMsg((imuPackage*) p_frame);
        }

        p_frame ++;
    }
}


int main(int argc, char* argv[])
{
    // Initialise the node
    ros::init(argc, argv, "arduino_node");
    ros::NodeHandle nh("~");

    int log_level = 0;
    nh.getParam("/arduino_log_level", log_level);
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, (ros::console::levels::Level) log_level)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    signal(SIGINT, SignalHandler);

    // Initialize driver class
    ArduinoDriver driver(nh);

    // Infinite loop
    ros::Rate rate(10); // Use a rate to avoid processor killing itself running in circles...
    while (ros::ok){
        // In loop, keep checking for messages
        ROS_INFO("Looping in Arduino driver");

        driver.checkSocket();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}