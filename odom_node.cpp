#include "odomter.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "odometer");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    Odometer odometer(nh, nhp);
    ros::spin();
    return 0;
}