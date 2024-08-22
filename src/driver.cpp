#include "driver.h"

void ArduinoDriver::parseBuffer(int buffer_size){
    char* p_end = buffer + buffer_size;
    char* p_frame = buffer;

    while (p_frame < p_end){
        //ROS_INFO("HEI");
        p_frame = (char*) memchr(p_frame, '$', p_end - p_frame);

        if (p_frame == nullptr){
            break;
        }

        else if (memcmp(p_frame, imu_header, 4) == 0 && (p_end - p_frame) >= sizeof(imuPackage)){
            imuData((imuPackage*) p_frame);
        }

        else if (memcmp(p_frame, gnss_header, 5) == 0 && (p_end - p_frame) >= sizeof(gnssPackage)){
            gnssData((gnssPackage*) p_frame);
        }

        p_frame ++;
    }
}


void ArduinoDriver::imuData(imuPackage* p_pkg){
    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time(p_pkg->t_sec, 1e3*p_pkg->t_usec);
    
    msg.linear_acceleration.x = 0.8*p_pkg->acc[0];
    msg.linear_acceleration.y = 0.8*p_pkg->acc[1];
    msg.linear_acceleration.z = 0.8*p_pkg->acc[2];

    msg.angular_velocity.x = 0.02*p_pkg->rate[0];
    msg.angular_velocity.y = 0.02*p_pkg->rate[1];
    msg.angular_velocity.z = 0.02*p_pkg->rate[2];

    imu_pub.publish(msg);
}

void ArduinoDriver::gnssData(gnssPackage* p_pkg){
    sensor_msgs::NavSatFix msg;
    msg.header.stamp = ros::Time(p_pkg->t_sec, 1e3*p_pkg->t_usec);
    
    msg.latitude = p_pkg->latitude;
    msg.longitude = p_pkg->longitude;
    msg.altitude = p_pkg->altitude;

    gnss_pub.publish(msg);
}

void ArduinoDriver::checkSocket(){
    int bytes_received; 
    while ((bytes_received = recv(sockfd, buffer, BUFFER_SIZE, 0)) > 0){
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
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);
    gnss_pub = nh.advertise<sensor_msgs::NavSatFix>("gnss", 10);
    status_pub = nh.advertise<arduino::arduino_status_msg>("status", 5);

    setupSocket();
    setupConfig();
}

ArduinoDriver::~ArduinoDriver(){
    config_service.shutdown();
}


void ArduinoDriver::setupConfig(){
    ROS_WARN("ArduinoDriver: Config not yet implemented");
}