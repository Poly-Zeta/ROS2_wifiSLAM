#include <rclcpp/rclcpp.hpp>
#include "image_binarizer/image_binarizer.hpp"

int main(int argc,char **argv){
    rclcpp::init(argc,argv);

    rclcpp::NodeOptions options;

    auto node=std::make_shared<ImageBinarizer>("image_binarizer",options);

    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

