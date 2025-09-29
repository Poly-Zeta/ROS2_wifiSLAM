#include "imu_odom_publisher/imu_odom_publisher.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
