#include "odom_publisher/static_tf_from_yaml.hpp"
#include <tf2/LinearMath/Quaternion.h>

StaticTfFromYaml::StaticTfFromYaml()
: Node("static_tf_from_yaml")
{
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // transforms の配列をインデックスで順に読む
    for (int i = 0; i < 10; ++i) {
        std::string prefix = "transforms." + std::to_string(i);

        std::string frame_id;
        if (!this->get_parameter(prefix + ".frame_id", frame_id)) {
            break; // これ以上要素がなければ終了
        }

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = frame_id;

        this->get_parameter(prefix + ".child_frame_id", t.child_frame_id);

        std::vector<double> trans;
        if (this->get_parameter(prefix + ".translation", trans) && trans.size() == 3) {
            t.transform.translation.x = trans[0];
            t.transform.translation.y = trans[1];
            t.transform.translation.z = trans[2];
        }

        std::vector<double> rpy;
        if (this->get_parameter(prefix + ".rotation_rpy", rpy) && rpy.size() == 3) {
            tf2::Quaternion q;
            q.setRPY(rpy[0], rpy[1], rpy[2]);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
        }

        broadcaster_->sendTransform(t);
        RCLCPP_INFO(this->get_logger(), "Published static TF: %s -> %s",
                    t.header.frame_id.c_str(), t.child_frame_id.c_str());
    }
}
