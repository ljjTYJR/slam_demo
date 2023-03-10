// Same as the static_transform_publisher node, send the static transform from
// the robot base to the sensor frame
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

struct StaticTransform {
    std::string frame_id;
    std::string child_frame_id;
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
};

void declareParams(rclcpp::Node::SharedPtr node) {
    node->declare_parameter<std::string>("robot_link", "base");
    node->declare_parameter<std::string>("sensor_link", "laser");
    node->declare_parameter<double>("x", 0.0);
    node->declare_parameter<double>("y", 0.0);
    node->declare_parameter<double>("z", 0.0);
    node->declare_parameter<double>("qx", 0.0);
    node->declare_parameter<double>("qy", 0.0);
    node->declare_parameter<double>("qz", 0.0);
    node->declare_parameter<double>("qw", 0.0);
}

void getParams(rclcpp::Node::SharedPtr node,
               StaticTransform& static_transform) {
    node->get_parameter("robot_link", static_transform.frame_id);
    node->get_parameter("sensor_link", static_transform.child_frame_id);
    node->get_parameter("x", static_transform.x);
    node->get_parameter("y", static_transform.y);
    node->get_parameter("z", static_transform.z);
    node->get_parameter("qx", static_transform.qx);
    node->get_parameter("qy", static_transform.qy);
    node->get_parameter("qz", static_transform.qz);
    node->get_parameter("qw", static_transform.qw);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sensor_offset_node");
    StaticTransform static_transform;
    declareParams(node);
    getParams(node, static_transform);
    tf2_ros::StaticTransformBroadcaster static_broadcaster(node);

    RCLCPP_INFO(node->get_logger(),
                "Publishing static transform from %s to %s",
                static_transform.frame_id.c_str(),
                static_transform.child_frame_id.c_str());
    RCLCPP_INFO(node->get_logger(),
                "x: %f, y: %f, z: %f",
                static_transform.x,
                static_transform.y,
                static_transform.z);
    RCLCPP_INFO(node->get_logger(),
                "qx: %f, qy: %f, qz: %f, qw: %f",
                static_transform.qx,
                static_transform.qy,
                static_transform.qz,
                static_transform.qw);

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.frame_id = static_transform.frame_id;
    transformStamped.child_frame_id = static_transform.child_frame_id;
    transformStamped.transform.translation.x = static_transform.x;
    transformStamped.transform.translation.y = static_transform.y;
    transformStamped.transform.translation.z = static_transform.z;
    transformStamped.transform.rotation.x = static_transform.qx;
    transformStamped.transform.rotation.y = static_transform.qy;
    transformStamped.transform.rotation.z = static_transform.qz;
    transformStamped.transform.rotation.w = static_transform.qw;

    static_broadcaster.sendTransform(transformStamped);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}