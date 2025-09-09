#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class StaticTfFromYaml : public rclcpp::Node
{
public:
    StaticTfFromYaml();

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

