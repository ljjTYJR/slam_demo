#include "ros/ros.h"
#include "slam_demo/OptSrv.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "client");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<slam_demo::OptSrv>("optimize_service", true);
    slam_demo::OptSrv srv;
    srv.request.req = 2;
    bool success = client.call(srv);

    if (success)
    {
        ROS_INFO("Response: %d", srv.response.res);
    }
    else
    {
        ROS_ERROR("Failed to call service my_service");
    }

    return 0;
}