#include "slam.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "slam");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    ROS_INFO("Slam node preares to initialize.");
    Slam slam(nh, nhp);
    ROS_INFO("Slam node initialized.");
    ros::spin();
    return 0;
}