// #include "odom_publisher/static_tf_from_yaml.hpp"
// #include <tf2/LinearMath/Quaternion.h>

// StaticTfFromYaml::StaticTfFromYaml()
// : Node("static_tf_from_yaml")
// {
//     broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

//     for (int i = 0; i < 50; ++i) {
//         std::string prefix = "transforms." + std::to_string(i);

//         // 事前に宣言（デフォルト値を設定）
//         declare_parameter<std::string>(prefix + ".frame_id", "");
//         declare_parameter<std::string>(prefix + ".child_frame_id", "");
//         declare_parameter<std::vector<double>>(prefix + ".translation", {0.0, 0.0, 0.0});
//         declare_parameter<std::vector<double>>(prefix + ".rotation_rpy", {0.0, 0.0, 0.0});

//         std::string frame_id;
//         get_parameter(prefix + ".frame_id", frame_id);
//         if (frame_id.empty()) {
//             break;
//         }

//         geometry_msgs::msg::TransformStamped t;
//         t.header.stamp = this->now();
//         t.header.frame_id = frame_id;

//         get_parameter(prefix + ".child_frame_id", t.child_frame_id);

//         std::vector<double> trans;
//         get_parameter(prefix + ".translation", trans);
//         if (trans.size() == 3) {
//             t.transform.translation.x = trans[0];
//             t.transform.translation.y = trans[1];
//             t.transform.translation.z = trans[2];
//         }

//         std::vector<double> rpy;
//         get_parameter(prefix + ".rotation_rpy", rpy);
//         if (rpy.size() == 3) {
//             tf2::Quaternion q;
//             q.setRPY(rpy[0], rpy[1], rpy[2]);
//             t.transform.rotation.x = q.x();
//             t.transform.rotation.y = q.y();
//             t.transform.rotation.z = q.z();
//             t.transform.rotation.w = q.w();
//         }

//         broadcaster_->sendTransform(t);
//         RCLCPP_INFO(this->get_logger(), "Published static TF: %s -> %s",
//                     t.header.frame_id.c_str(), t.child_frame_id.c_str());
//     }

// }

#include "odom_publisher/static_tf_from_yaml.hpp"
#include <tf2/LinearMath/Quaternion.h>

StaticTfFromYaml::StaticTfFromYaml()
: Node("static_tf_from_yaml")
{
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // 0, 1, 2, ... の番号キーを順に読む
    for (int i = 0; i < 50; ++i) {
        std::string prefix = std::to_string(i) + ".transforms";

        // 事前に宣言（デフォルト値を設定）
        declare_parameter<std::string>(prefix + ".frame_id", "");
        declare_parameter<std::string>(prefix + ".child_frame_id", "");
        declare_parameter<std::vector<double>>(prefix + ".translation", {0.0, 0.0, 0.0});
        declare_parameter<std::vector<double>>(prefix + ".rotation_rpy", {0.0, 0.0, 0.0});

        std::string frame_id;
        get_parameter(prefix + ".frame_id", frame_id);
        if (frame_id.empty()) {
            // この番号のframe_idが無ければ終了
            break;
        }

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = frame_id;

        get_parameter(prefix + ".child_frame_id", t.child_frame_id);

        std::vector<double> trans;
        get_parameter(prefix + ".translation", trans);
        if (trans.size() == 3) {
            t.transform.translation.x = trans[0];
            t.transform.translation.y = trans[1];
            t.transform.translation.z = trans[2];
        }

        std::vector<double> rpy;
        get_parameter(prefix + ".rotation_rpy", rpy);
        if (rpy.size() == 3) {
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