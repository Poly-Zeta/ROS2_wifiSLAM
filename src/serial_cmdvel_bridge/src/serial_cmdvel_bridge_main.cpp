#include "serial_cmdvel_bridge/serial_cmdvel_bridge.hpp"
#include <rclcpp/rclcpp.hpp>


int main(int argc,char **argv){

    rclcpp::init(argc,argv);
    rclcpp::NodeOptions node_options;

    auto node=std::make_shared<SerialCmdvelBridge>("serial_cmdvel_bridge_node",node_options);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}
