#include "rclcpp/rclcpp.hpp"
#include "slam_demo/OptSrv.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("client_node");
    auto client = node->create_client<slam_demo::OptSrv>("opt_client");
    slam_demo::OptSrv srv;
    srv.request.req = 2;
    bool success = client.call(srv);
    return 0;
}