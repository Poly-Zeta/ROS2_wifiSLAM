#include "imu_odom_publisher/imu_odom_publisher.hpp"

ImuOdomPublisher::ImuOdomPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("imu_odom_publisher", options) {
    imu_topic_   = this->declare_parameter<std::string>("imu_topic", "i2cReceive/imu");
    odom_topic_  = this->declare_parameter<std::string>("odom_topic", "/imu_odom");
    odom_frame_  = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_  = this->declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_  = this->declare_parameter<bool>("publish_tf", true);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ImuOdomPublisher::imuCallback, this, std::placeholders::_1)
    );
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    if (publish_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }
}

void ImuOdomPublisher::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = msg->orientation;

    for (int i = 0; i < 36; ++i){
        odom.pose.covariance[i] = 0.0;
    }
    odom.pose.covariance[0]  = 1e3;
    odom.pose.covariance[7]  = 1e3;
    odom.pose.covariance[14] = 1e3;
    odom.pose.covariance[21] = 5e-3;
    odom.pose.covariance[28] = 5e-3;
    odom.pose.covariance[35] = 2e-3;

    odom_pub_->publish(odom);

    if (publish_tf_ && tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = odom.header.stamp;
        tf.header.frame_id = odom_frame_;
        tf.child_frame_id = base_frame_;
        tf.transform.translation.x = 0.0;
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
    }
}