#ifndef __VISUALIZATION_H
#define __VISUALIZATION_H

#include "types.h"
#include <ros/ros.h>
#include <string>

class Visualization {
public:
    Visualization(const ros::NodeHandle& nh);
    ~Visualization() {};

    void publishLineOfTwoPoses(const MatrixSE2& pose1, const MatrixSE2& pose2, const std::string& frame_id, const double& duration);
private:
    ros::NodeHandle nh_;
    ros::Publisher  pub_line_;

    std::string pub_line_topic_;
};

#endif // __VISUALIZATION_H