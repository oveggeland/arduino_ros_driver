#include "control.h"

/*
This function is used to publish the arduino status on a regular basis from the main while loop
*/
void publish_status(ros::Publisher* status_pub){
    arduino::arduino_status_msg status_msg;

    if (status_pub->getNumSubscribers() > 0){
        status_msg.header.stamp = ros::Time::now();
        status_pub->publish(status_msg); 
    }
};

/*
This defines the ROS service responsible for configuration of the Arduino during operation.
*/
bool config_service_cb(arduino::arduino_config_srv::Request &req, arduino::arduino_config_srv::Response &res){
    try{ // Reboot
        ROS_INFO("Config service for Arduino");
    } 
    catch(...){
        ROS_ERROR_STREAM("Config service for Arduino failed");
    }

    return true;
};