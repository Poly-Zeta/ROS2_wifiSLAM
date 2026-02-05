#include "line_follower/line_follower.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

LineFollower::LineFollower(const std::string & node_name,const rclcpp::NodeOptions & node_options) : rclcpp::Node(node_name,node_options){
    kp_=declare_parameter<double>("kp",0.1);
    ka_=declare_parameter<double>("ka",0.05);
    speed_=declare_parameter<double>("speed",0.1);


    bin_image_topic_ =this->declare_parameter<std::string>("bin_image_topic","/image_binary");

    cmd_vel_topic_ =this->declare_parameter<std::string>("cmd_vel_topic","/cmd_vel");

    bin_image_sub_=this->create_subscription<sensor_msgs::msg::Image>(
        bin_image_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(
            &LineFollower::lineFollowerCallback,
            this,
            std::placeholders::_1
        )
    );

    cmd_vel_pub_=this->create_publisher<geometry_msgs::msg::Twist>(
        cmd_vel_topic_,
        rclcpp::QoS(10)
    );
}

void LineFollower::lineFollowerCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg){

    const int img_width=msg->width;
    const int img_height=msg->height;

    const int center_x=img_width/2;

    geometry_msgs::msg::Twist cmd;

    cv::Mat cv_bin_image =cv_bridge::toCvCopy(msg,"mono8")->image;
    cv::Mat cv_inv_bin_image=cv_bin_image.clone();
    //cv::Mat cv_inv_bin_image;
//    cv::bitwise_not(cv_bin_image,cv_inv_bin_image);

    int h=cv_inv_bin_image.rows;
    int w=cv_inv_bin_image.cols;

    cv::Mat roi_bottom=cv_inv_bin_image(
        cv::Range(h*2.0/3.0,h),
        cv::Range::all()
    );

    cv::Mat roi_top=cv_inv_bin_image(
        cv::Range(h/3,h*2.0/3.0),
        cv::Range::all()
    );

    cv::Moments m_top = cv::moments(roi_top,true);
    cv::Moments m_bottom = cv::moments(roi_bottom,true);

    cmd.linear.x=0.0;
    cmd.angular.z=0.0;

    const bool detected = (m_top.m00 != 0 && m_bottom.m00 != 0); 

    if(detected){
        lost_count_ = 0;

        double cx_top=m_top.m10/m_top.m00;
        double cx_bottom=m_bottom.m10/m_bottom.m00;

        double pos_error   = (cx_bottom - center_x) /(double)center_x;
        double angle_error = (cx_bottom - cx_top)   /(double)center_x;

        pos_error_f_   = (1.0-alpha_) * pos_error_f_   + alpha_ * pos_error;
        angle_error_f_ = (1.0-alpha_) * angle_error_f_ + alpha_ * angle_error;

        cmd.linear.x = speed_;
        cmd.angular.z = -kp_ * pos_error - ka_ * angle_error;

        last_angular_z_=cmd.angular.z;
    }else{
        lost_count_+=1;

        if(lost_count_<lost_debounce_frames_){
            cmd.linear.x  = speed_;
            cmd.angular.z = last_angular_z_;
        }else{
            cmd.linear.x  = speed_ * lost_linear_scale_;
            cmd.angular.z = search_dir_ * w_search_;
            
            if(search_flip_period_ > 0 && (lost_count_ % search_flip_period_ ==0)){
                search_dir_ *= -1.0;
            }
            last_angular_z_=cmd.angular.z;
        }
    }

    cmd_vel_pub_->publish(cmd);
    return;
}
