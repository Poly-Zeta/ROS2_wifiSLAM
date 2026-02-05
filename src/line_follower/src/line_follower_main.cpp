#include <rclcpp/rclcpp.hpp>
#include "line_follower/line_follower.hpp"

int main(int argc,char **argv){
    rclcpp::init(argc,argv);

    rclcpp::NodeOptions options;

    auto node=std::make_shared<LineFollower>("line_follower",options);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
