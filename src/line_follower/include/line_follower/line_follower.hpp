#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>

class LineFollower : public rclcpp::Node{
    public:
        LineFollower(const std::string & node_name,const rclcpp::NodeOptions & node_options);

    private:
        void lineFollowerCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

        std::string bin_image_topic_;
        std::string cmd_vel_topic_;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr bin_image_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

        double last_angular_z_=0.0;

        double pos_error_f_   = 0.0;
        double angle_error_f_ = 0.0;
        double alpha_         = 0.2;
        
        int    lost_count_           = 0;
        int    lost_debounce_frames_ = 3;
        double lost_linear_scale_    = 0.10;
        double w_search_             = 0.6;
        double search_dir_           = 1.0;
        int    search_flip_period_   = 30;


        //parameters
        double kp_{0.1};
        double ka_{0.05};
        double speed_{0.1};
};
