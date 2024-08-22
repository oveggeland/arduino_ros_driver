#include "ros/ros.h"

#include "driver.h"

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

using namespace std;

inline void SignalHandler(int signum) {
  ros::shutdown();
  exit(signum);
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
    ros::Rate rate(100); // Use a rate to avoid processor killing itself running in circles...
    while (ros::ok){
        // In loop, keep checking for messages
        driver.checkSocket();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}