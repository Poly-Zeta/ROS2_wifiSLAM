#include "odom_publisher/odom_publisher.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto node = std::make_shared<OdomPublisher>(options);

    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
