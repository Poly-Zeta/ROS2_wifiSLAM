#include "odom_publisher/static_tf_from_yaml.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTfFromYaml>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
