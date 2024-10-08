#include "driver.h"

void ArduinoDriver::parseBuffer(int buffer_size){
    char* p_end = buffer + buffer_size;
    char* p_frame = buffer;

    while (p_frame < p_end){
        p_frame = (char*) memchr(p_frame, '$', p_end - p_frame);

        if (p_frame == nullptr){
            break;
        }

        else if (memcmp(p_frame, imu_header, 4) == 0 && (p_end - p_frame) >= sizeof(imuPackage)){
            imuData((imuPackage*) p_frame);
            t_last_contact = ros::Time::now();
        }

        else if (memcmp(p_frame, gnss_header, 5) == 0 && (p_end - p_frame) >= sizeof(gnssPackage)){
            gnssData((gnssPackage*) p_frame);
            t_last_contact = ros::Time::now();
        }

        else if (memcmp(p_frame, status_header, 3) == 0 && (p_end - p_frame) >= sizeof(arduinoStatus)){
            statusUpdate((arduinoStatus*) p_frame);
            t_last_contact = ros::Time::now();
        }

        p_frame ++;
    }
}


void ArduinoDriver::imuData(imuPackage* p_pkg){
    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time(p_pkg->t_sec, 1e3*p_pkg->t_usec);
    
    msg.linear_acceleration.x = MG2MS2*0.8*p_pkg->acc[0];
    msg.linear_acceleration.y = MG2MS2*0.8*p_pkg->acc[1];
    msg.linear_acceleration.z = MG2MS2*0.8*p_pkg->acc[2];

    msg.angular_velocity.x = DEG2RAD*0.02*p_pkg->rate[0];
    msg.angular_velocity.y = DEG2RAD*0.02*p_pkg->rate[1];
    msg.angular_velocity.z = DEG2RAD*0.02*p_pkg->rate[2];

    imu_pub.publish(msg);
}

void ArduinoDriver::gnssData(gnssPackage* p_pkg){
    sensor_msgs::NavSatFix msg;
    msg.header.stamp = ros::Time(p_pkg->t_sec, 1e3*p_pkg->t_usec);
    
    msg.latitude = p_pkg->latitude/1.0e7;
    msg.longitude = p_pkg->longitude/1.0e7;
    msg.altitude = p_pkg->altitude/1000.0;

    gnss_pub.publish(msg);
}

void ArduinoDriver::statusUpdate(arduinoStatus* p_pkg){
    arduino::arduino_status_msg msg;
    msg.header.stamp = ros::Time(p_pkg->t_sec, 1e3*p_pkg->t_usec);
    
    msg.st_arduino_connection = true;
    msg.st_arduino_ip_addr = p_pkg->ip;

    msg.st_arduino_age = p_pkg->age;
    msg.st_arduino_sync_offset = p_pkg->sync_offset;

    msg.st_arduino_imu_rate = p_pkg->imu_rate;
    msg.st_arduino_imu_id = p_pkg->imu_id;
    msg.st_arduino_imu_temp = p_pkg->imu_temp;

    msg.st_arduino_gnss_reset = p_pkg->gnss_reset;
    msg.st_arduino_gnss_reset_cnt = p_pkg->gnss_reset_cnt;

    msg.st_arduino_ptp_interval = p_pkg->ptp_interval;

    status_pub.publish(msg);
}

void ArduinoDriver::checkSocket(){
    int bytes_received; 
    while ((bytes_received = recv(sockfd, buffer, BUFFER_SIZE, 0)) > 0){
        parseBuffer(bytes_received);
    }
}

void ArduinoDriver::checkTimeout(){
    if (ros::Time::now() - t_last_contact > CONNECTION_TIMEOUT){
        arduino::arduino_status_msg msg;
        status_pub.publish(msg); // Send empty status message, indicating a timeout
        t_last_contact = ros::Time::now(); // Reset this to not overflow the network with empty status messages
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
    
    config_service = nh.advertiseService<
        arduino::arduino_config_srv::Request, 
        arduino::arduino_config_srv::Response
        >("config", &ArduinoDriver::configService);
}

ArduinoDriver::~ArduinoDriver(){
    config_service.shutdown();
}


bool ArduinoDriver::configService(arduino::arduino_config_srv::Request &req, arduino::arduino_config_srv::Response &res){
    arduino::arduino_config_msg msg = req.config_msg;
    arduinoCommand cmd;
    
    cmd.reset = msg.reset;
    cmd.ntp_interval = msg.ntp_interval;
    cmd.ptp_active = msg.ptp_active;
    cmd.ptp_interval = msg.ptp_interval;
    cmd.imu_active = msg.imu_active;
    cmd.imu_sr = msg.imu_rate;
    cmd.gnss_active = msg.gnss_active;
    cmd.gnss_sr = msg.gnss_rate;

    return true;
}