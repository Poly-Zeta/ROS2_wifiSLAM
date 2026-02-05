#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

class ImageBinarizer : public rclcpp::Node{
    public:
        ImageBinarizer(const std::string & node_name,const rclcpp::NodeOptions & node_options);
    
    private:
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

        std::string image_topic_;
        std::string binImage_topic_;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr binImage_pub_;
};

