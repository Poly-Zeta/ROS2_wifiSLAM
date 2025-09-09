#include "i2c_sensors_driver/i2c_sensors_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    
    auto node = std::make_shared<I2CsensorsDriver>("i2c_sensors_driver_node", node_options);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}