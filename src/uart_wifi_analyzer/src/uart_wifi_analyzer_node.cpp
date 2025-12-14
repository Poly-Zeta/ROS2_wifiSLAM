#include "uart_wifi_analyzer/uart_wifi_analyzer.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sstream>


UartWifiAnalyzer::UartWifiAnalyzer(const rclcpp::NodeOptions & options)
: Node("uart_wifi_analyzer", options){
    device_   = this->declare_parameter<std::string>("device",  "/dev/ttyUSB-ESP32C5-WifiAnalyzer");
    baudrate_ = this->declare_parameter<int>("baudrate", 115200);

    uart_wifi_pub_RSSI_ = this->create_publisher<std_msgs::msg::String>("uart_wifi/RSSI", 10);
    uart_wifi_pub_FTM_ = this->create_publisher<std_msgs::msg::String>("uart_wifi/FTM", 10);

    RCLCPP_INFO(this->get_logger(), "UART node initialized. device=%s baud=%d",
            device_.c_str(), baudrate_);
}

int UartWifiAnalyzer::openSerial(){
    int fd = open(device_.c_str(), O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open UART device: %s", device_.c_str());
        return -1;
    }

    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, baudrate_);
    cfsetispeed(&tty, baudrate_);
    tty.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd, TCSANOW, &tty);

    return fd;
}

void UartWifiAnalyzer::processLine(const std::string & line){
    std::stringstream ss(line);
    std::string token;
    std::getline(ss, token, ',');

    token.erase(std::remove(token.begin(), token.end(), '\r'), token.end());
    token.erase(std::remove(token.begin(), token.end(), '\n'), token.end());

    std_msgs::msg::String msg;
    msg.data = line;

    if (token == "0") {
        uart_wifi_pub_RSSI_->publish(msg);
        // RCLCPP_DEBUG(this->get_logger(), "Published to type0: %s", line.c_str());
    } else if (token == "1") {
        uart_wifi_pub_FTM_->publish(msg);
        // RCLCPP_DEBUG(this->get_logger(), "Published to type1: %s", line.c_str());
    } else {
        // RCLCPP_WARN(this->get_logger(), "Unknown line format: %s", line.c_str());
    }
}

void UartWifiAnalyzer::run(){
    int fd = openSerial();
    if (fd < 0){
        return;
    }

    std::string buffer;
    char c;
    while (rclcpp::ok()) {
        int n = read(fd, &c, 1);
        if (n > 0) {
        if (c == '\n' || c == '\r') {
            if (!buffer.empty()) {
                processLine(buffer);
                buffer.clear();
            }
        } else {
            buffer.push_back(c);
        }
        }
        rclcpp::spin_some(shared_from_this());
    }
    close(fd);
}

