#include "serial_cmdvel_bridge/serial_cmdvel_bridge.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cstdio>
#include <cstring>
#include <stdexcept>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

SerialCmdvelBridge::SerialCmdvelBridge(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options),fd_(-1){

    cmdvel_topic_ =this->declare_parameter<std::string>("cmdvel_topic","/cmd_vel");

    cmdvel_sub_=this->create_subscription<geometry_msgs::msg::Twist>(
        cmdvel_topic_,
        10,
        std::bind(
            &SerialCmdvelBridge::cmdvelCallback,
            this,
            std::placeholders::_1
        )
    );

    char device_name[] = "/dev/ttyUSB-ESP32-myRobot";
    fd_ = open_serial(device_name);

}


SerialCmdvelBridge::~SerialCmdvelBridge(){
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}


int SerialCmdvelBridge::open_serial(const char *device_name){
    int fd = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    fcntl(fd, F_SETFL, 0);

    if (fd < 0) {
        throw std::runtime_error("failed to open serial");
    }

    struct termios conf_tio;
    tcgetattr(fd, &conf_tio);

    speed_t BAUDRATE = B230400;
    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);

    conf_tio.c_lflag &= ~(ECHO | ICANON);

    conf_tio.c_cc[VMIN] = 0;
    conf_tio.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &conf_tio);
    return fd;
}

void SerialCmdvelBridge::cmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
    char buf[64];
    int bytes_written;

    //RCLCPP_INFO(this->get_logger(), "I heard: '%lf'", msg->linear.x);

    bytes_written = snprintf(buf, sizeof(buf), "%7.3f,%7.3f,%7.3f,%7.3f\n", msg->linear.x, msg->angular.z, msg->angular.x, msg->angular.y);


    if (bytes_written<0 || (int)bytes_written > sizeof(buf)) {
        RCLCPP_ERROR(this->get_logger(), "Serial Fail: message formatting error");
        return;
    }else{
        printf("cmd_vel recv:%s\n", buf);

        int rec = write(fd_, buf, bytes_written);

        if (rec >= 0) {
            printf("Serial send:%s\n", buf);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Serial Fail: could not write");
        }
    }
}

