#pragma once

#include "ros/ros.h"
#include <ros/package.h>

#include "arduino/arduino_status_msg.h"
#include "arduino/arduino_config_srv.h"

#include <iostream>
#include <boost/algorithm/string.hpp>

using namespace std;

void publish_status(ros::Publisher* status_pub);
bool config_service_cb(arduino::arduino_config_srv::Request &req, arduino::arduino_config_srv::Response &res);