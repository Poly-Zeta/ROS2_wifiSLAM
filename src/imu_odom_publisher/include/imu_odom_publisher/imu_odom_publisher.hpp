#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class ImuOdomPublisher : public rclcpp::Node {
    public:
        explicit ImuOdomPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    private:
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        std::string imu_topic_;
        std::string odom_topic_;
        std::string odom_frame_;
        std::string base_frame_;
        bool publish_tf_{true};
};