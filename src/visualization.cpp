#include "slam_demo/visualization.h"
#include <rclcpp/time_source.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/clock.hpp>

Visualization::Visualization(const rclcpp::Node::SharedPtr node) {
    node_ = node;
    pub_line_ = node_->create_publisher<visualization_msgs::msg::Marker>("loop_marker", 10);
}

/**
 * receive two poses and draw two points and a line between them
*/
void Visualization::publishLineOfTwoPoses(const MatrixSE2& pose1, const MatrixSE2& pose2, const std::string& frame_id, const double& duration) {

    static visualization_msgs::msg::Marker points, line;
    points.header.frame_id = line.header.frame_id = frame_id;
    points.header.stamp = line.header.stamp = rclcpp::Clock().now();
    // TODO: to decide the namespace
    points.ns = line.ns = "visualization";
    points.action = line.action = visualization_msgs::msg::Marker::ADD;
    points.pose.orientation.w = line.pose.orientation.w = 1.0;

    points.id = 0;
    line.id = 1;

    points.type = visualization_msgs::msg::Marker::POINTS;
    line.type = visualization_msgs::msg::Marker::LINE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line.scale.x = 0.025;

    // Points are green
    points.color.g = 0.5f;
    points.color.a = 0.5f;

    // Line is blue
    line.color.r = 0.5f;
    line.color.a = 0.5f;

    geometry_msgs::msg::Point p1, p2;
    p1.x = pose1(0, 2);
    p1.y = pose1(1, 2);
    p1.z = 0;
    p2.x = pose2(0, 2);
    p2.y = pose2(1, 2);
    p2.z = 0;

    points.points.push_back(p1);
    points.points.push_back(p2);
    line.points.push_back(p1);
    line.points.push_back(p2);
    points.lifetime = line.lifetime = rclcpp::Duration::from_seconds(duration);

    pub_line_->publish(points);
    pub_line_->publish(line);

    return;
}