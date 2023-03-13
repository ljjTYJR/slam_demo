#include "ros/ros.h"
#include "slam_demo/OptSrv.h"

bool my_service_callback(slam_demo::OptSrv::Request& req, slam_demo::OptSrv::Response& res) {
    res.res = req.req;
    ROS_INFO("request: x=%d", (int)req.req);
    ROS_INFO("sending back response: %d", res.res);
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "service_server");
    ros::NodeHandle n;

    ros::ServiceServer server = n.advertiseService("my_service", my_service_callback);
    ROS_INFO("Ready to receive requests");
    ros::spin();

    return 0;
}