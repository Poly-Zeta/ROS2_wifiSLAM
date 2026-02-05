#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>

class SerialCmdvelBridge : public rclcpp::Node{
    public:
        SerialCmdvelBridge(const std::string & node_name,const rclcpp::NodeOptions & node_options);
	~SerialCmdvelBridge();

    private:

        void cmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
	int open_serial(const char *device_name);

	int fd_;

        std::string cmdvel_topic_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub_;
};
