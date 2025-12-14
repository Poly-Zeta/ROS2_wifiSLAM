#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

class UartWifiAnalyzer : public rclcpp::Node {
    public:
        explicit UartWifiAnalyzer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        void run();
    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr uart_wifi_pub_RSSI_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr uart_wifi_pub_FTM_;

        std::string device_;
        int baudrate_;

        int openSerial();
        void processLine(const std::string & line);
};