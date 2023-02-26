#pragma once

// customized
#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"
#include "types.h"
// Standard C/C++
#include <vector>
#include <utility>
// Third party
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using RingKeyVecOfVecs = std::vector<std::vector<float> >;
using RingKeyKDTree = KDTreeVectorOfVectorsAdaptor< RingKeyVecOfVecs, float >;
double deg2rad(double degrees);

class ScanContextManger {

    typedef struct POINT_CLOUD_DSC {
        unsigned int id;
        Eigen::MatrixXf scan_context;
        std::vector<float> ring_key;                        // The ring key should be one-dimensional vector
        // pcl::PointCloud<pcl::PointXY>::Ptr cloud;    //TODO: decide whether to store the cloud, in the odometer or here
    }POINT_CLOUD_DSC_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    ScanContextManger();
    ~ScanContextManger();

    // The user API
        std::pair<int, double> addNewFrame(const unsigned int id,
                                           const pcl::PointCloud<pcl::PointXY>::Ptr cloud);
        /* TODO: add the the `addNewFrame` function? */
        bool isPossibleLoop(const MatrixSE2& pose1, const MatrixSE2& pose2);

private:

    // The used internal functions
        // The scan context generation
        Eigen::MatrixXf generateScanContext(const pcl::PointCloud<pcl::PointXY>::Ptr cloud);
        // The ring key generation
        std::vector<float> generateRingKey(const Eigen::MatrixXf& scan_context);
        // measure the similarity between two scan contexts
        std::pair<double, int> scanContextAlignment(Eigen::MatrixXf& q_scan_context, Eigen::MatrixXf& ref_scan_context);
        Eigen::MatrixXf scanContextMatShift(Eigen::MatrixXf& scan_context, int shift);
        std::pair<int, double> detectLoopClosure(const POINT_CLOUD_DSC_& cur_dsc);
        double distDirectSC(Eigen::MatrixXf& q_scan_context, Eigen::MatrixXf& ref_scan_context);

    // The parameters setting (k:constant)
    // TODO: the hyper-parameters need to be tuned
        const unsigned int kNumRings = 20;
        const unsigned int kNumSector = 60;
        const unsigned int kSkipLaestFrames = 25;   // To detect the loop closure, we skip latest `kSkipLaestFrames` frames
        const unsigned int kNumRingKeyQuery = 10;        // The number of key frames(ring key) to query
        const double kLoopDistThres = 0.3;                // The maximum distance between two possbile poses
        const double kMaxRadius = 10.0;             // meter
        const double kSectorResolution = 360.0 / (double)(kNumSector);
        const double kRingResolution = kMaxRadius / (double)(kNumRings);
        const double kSC_DIST_THRES = 0.5;        // The threshold of the distance between two scan contexts (get from original implementation)

    // The data-structure maintained by the scan context manager
        // Todo: decide the variable types
        std::vector<POINT_CLOUD_DSC_> point_cloud_dsc_buffer_;
        RingKeyVecOfVecs ring_key_buffer_;      // save all the ring keys
        RingKeyVecOfVecs tmp_ring_key_search_base_; // A temporary variable to store the ring keys for the KDTree
        std::unique_ptr<RingKeyKDTree> ring_key_kd_tree_;
};

