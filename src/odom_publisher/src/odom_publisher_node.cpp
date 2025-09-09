#include "odom_publisher/odom_publisher.hpp"
#include <cmath>
#include <functional>

OdomPublisher::OdomPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("odom_publisher", options)
{
    // Declare parameters (overridden by YAML)
    declare_parameter<std::string>("left_topic",  "/i2cReceive/l_wheel_count");
    declare_parameter<std::string>("right_topic", "/i2cReceive/r_wheel_count");
    declare_parameter<std::string>("odom_frame",  "odom");
    declare_parameter<std::string>("base_frame",  "base_link");
    declare_parameter<bool>("publish_tf", true);
    declare_parameter<double>("dist_per_count", 7.9353e-5); // m/count
    declare_parameter<double>("tread",          0.155);     // m

    // Get parameters
    get_parameter("left_topic",  left_topic_);
    get_parameter("right_topic", right_topic_);
    get_parameter("odom_frame",  odom_frame_);
    get_parameter("base_frame",  base_frame_);
    get_parameter("publish_tf",  publish_tf_);
    get_parameter("dist_per_count", dist_per_count_);
    get_parameter("tread",          tread_);

    // QoS for sensor data
    auto qos = rclcpp::SensorDataQoS();

    sub_left_ = create_subscription<std_msgs::msg::Int32>(
        left_topic_, qos,
        std::bind(&OdomPublisher::leftCountCallback, this, std::placeholders::_1));

    sub_right_ = create_subscription<std_msgs::msg::Int32>(
        right_topic_, qos,
        std::bind(&OdomPublisher::rightCountCallback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS{50});
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_time_ = now();

    RCLCPP_INFO(get_logger(),
        "odom_publisher configured:\n  left_topic=%s\n  right_topic=%s\n  dist_per_count=%.9f m\n  tread=%.3f m",
        left_topic_.c_str(), right_topic_.c_str(), dist_per_count_, tread_);
}

void OdomPublisher::leftCountCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    left_count_ = msg->data;
    got_left_ = true;
    maybeUpdate();
}

void OdomPublisher::rightCountCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    right_count_ = msg->data;
    got_right_ = true;
    maybeUpdate();
}

int64_t OdomPublisher::deltaTicks(int32_t cur, int32_t prev) {
    // Compute smallest signed delta on 32-bit ring
    uint32_t ucur = static_cast<uint32_t>(cur);
    uint32_t uprev = static_cast<uint32_t>(prev);
    uint32_t udiff = ucur - uprev; // modulo 2^32
    int64_t diff = static_cast<int64_t>(udiff);
    if (diff > static_cast<int64_t>(1ULL << 31)){
        diff -= static_cast<int64_t>(1ULL << 32);
    }
    return diff;
}

void OdomPublisher::maybeUpdate() {
    if (!(got_left_ && got_right_)) {
        return;
    }

    const rclcpp::Time t = now();
    const double dt = (t - last_time_).seconds();

    if (first_update_ || dt <= 0.0) {
        first_update_ = false;
        last_time_ = t;
        prev_left_  = left_count_;
        prev_right_ = right_count_;
        return;
    }

    const int64_t dL_ticks = deltaTicks(left_count_,  prev_left_);
    const int64_t dR_ticks = deltaTicks(right_count_, prev_right_);

    prev_left_  = left_count_;
    prev_right_ = right_count_;
    last_time_ = t;

    // Distances
    const double dL = static_cast<double>(dL_ticks) * dist_per_count_;
    const double dR = static_cast<double>(dR_ticks) * dist_per_count_;
    const double ds = 0.5 * (dL + dR);
    const double dth = (dR - dL) / tread_;

    // Integrate (2nd order)
    x_   += ds * std::cos(yaw_ + 0.5 * dth);
    y_   += ds * std::sin(yaw_ + 0.5 * dth);
    yaw_ += dth;

    const double v  = ds / dt;
    const double wz = dth / dt;

    // Odometry msg
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id  = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(yaw_ * 0.5);
    odom.pose.pose.orientation.w = std::cos(yaw_ * 0.5);

    // Minimal covariances (tunable; non-zero helps some consumers)
    odom.pose.covariance[0]  = 1e-3; // x
    odom.pose.covariance[7]  = 1e-3; // y
    odom.pose.covariance[35] = 5e-3; // yaw

    odom.twist.twist.linear.x  = v;
    odom.twist.twist.linear.y  = 0.0;
    odom.twist.twist.angular.z = wz;
    odom.twist.covariance[0]  = 1e-2; // vx
    odom.twist.covariance[35] = 5e-2; // wz

    odom_pub_->publish(odom);

    if (publish_tf_) {
        geometry_msgs::msg::TransformStamped tfm;
        tfm.header.stamp = t;
        tfm.header.frame_id = odom_frame_;
        tfm.child_frame_id  = base_frame_;
        tfm.transform.translation.x = x_;
        tfm.transform.translation.y = y_;
        tfm.transform.translation.z = 0.0;
        tfm.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tfm);
    }
}