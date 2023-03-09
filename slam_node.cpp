#include "rclcpp/rclcpp.hpp"
#include "slam.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("slam_node");
    Slam slam(node);
    rclcpp::spin(node);
    return 0;
}