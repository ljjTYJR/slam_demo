#ifndef __ICP_H__
#define __ICP_H__

class icp {
public:
    icp();
    ~icp();

    void registerPointClouds(const PointCloud& source, const PointCloud& target, const Eigen::Matrix4d& init_guess, Eigen::Matrix4d& final_guess);
}






#endif /* __ICP_H__ */