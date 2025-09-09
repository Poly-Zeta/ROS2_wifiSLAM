#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdomPublisher : public rclcpp::Node {
    public:
        explicit OdomPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    private:
        // Callbacks
        void leftCountCallback(const std_msgs::msg::Int32::SharedPtr msg);
        void rightCountCallback(const std_msgs::msg::Int32::SharedPtr msg);
        void maybeUpdate();

        // Safe delta for 32-bit wrap-around counters
        static int64_t deltaTicks(int32_t cur, int32_t prev);

        // Subscriptions
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_left_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_right_;

        // Publications
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // Latest and previous counts
        int32_t left_count_{0}, right_count_{0};
        int32_t prev_left_{0}, prev_right_{0};
        bool got_left_{false}, got_right_{false};
        bool first_update_{true};

        // Robot state
        double x_{0.0}, y_{0.0}, yaw_{0.0};
        rclcpp::Time last_time_;

        // Parameters
        std::string left_topic_;
        std::string right_topic_;
        std::string odom_frame_;
        std::string base_frame_;
        bool publish_tf_{true};
        double dist_per_count_{7.9353e-5}; // m/count
        double tread_{0.155};              // m
};
