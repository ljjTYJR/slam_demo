#include "slam.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "slam");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    Slam slam(nh, nhp);
    ros::spin();
    return 0;
}