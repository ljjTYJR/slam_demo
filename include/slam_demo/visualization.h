#pragma once
// cunstomed
#include "types.h"
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker.hpp>
// standard C/C++
#include <string>

class Visualization {
   public:
    Visualization(const rclcpp::Node::SharedPtr node);
    ~Visualization(){};

    void publishLineOfTwoPoses(const MatrixSE2& pose1,
                               const MatrixSE2& pose2,
                               const std::string& frame_id,
                               const double& duration);

   private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_line_;
    std::string pub_line_topic_;
};
