#include "scan_context.h"

// ROS2
#include "rclcpp/rclcpp.hpp"

// standard C/C++
#include <math.h>
#include <limits>
#include <chrono>
#include <cmath>
#include <memory>

/**
 * Convert degrees to radians, the input angle is in range [0, 360], the output angle is in range [0, 2π]
 */
double deg2rad(double degrees) {
    double radians = M_PI / 180.0 * degrees;
    // Normalize radians to range [0, 2π]
    double normalized_radians = fmod(radians, 2*M_PI);
    if (normalized_radians < 0) {
        normalized_radians += 2*M_PI;
    }
    return normalized_radians;
}

ScanContextManger::ScanContextManger() {
    RCLCPP_INFO(rclcpp::get_logger("ScanContextManger"), "ScanContextManger is initialized");
    return;
}
ScanContextManger::~ScanContextManger() { }

// The internal helper functions
Eigen::MatrixXf ScanContextManger::generateScanContext(const pcl::PointCloud<pcl::PointXY>::Ptr cloud) {
    unsigned int num_points = cloud->size();
    assert(num_points > 0);
    float azim_angle = 0, azim_range = 0;
    int ring_idx = 0, sctor_idx = 0;
    Eigen::MatrixXf scan_context = Eigen::MatrixXf::Zero(kNumRings, kNumSector);

    for (unsigned int i = 0; i < num_points; i++) {
        azim_angle = atan2(cloud->points[i].y, cloud->points[i].x) * 180.0 / M_PI + 180.0; // [0, 360]
        azim_range = sqrt(cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y);
        if (azim_range > kMaxRadius) {
            // The point is out of the range, regard it as a noise
            continue;
        }
        // lower the bound of the index
        ring_idx = (int)(azim_range / kRingResolution);
        sctor_idx = (int)(azim_angle / kSectorResolution);
        if (ring_idx >= kNumRings) {
            ring_idx = kNumRings - 1;
        }
        if (sctor_idx >= kNumSector) {
            sctor_idx = kNumSector - 1;
        }
        scan_context(ring_idx, sctor_idx) += 1;
    }

    return scan_context;
}

std::vector<float> ScanContextManger::generateRingKey(const Eigen::MatrixXf& scan_context) {
    // ring key is used for quick query, the requirement is that the ring key should be rotation invariant
    std::vector<float> ring_key = std::vector<float>(kNumRings, 0);
    for (unsigned int i = 0; i < kNumRings; i++) {
        ring_key[i] = scan_context.row(i).sum();
    }
    return ring_key;
}

Eigen::MatrixXf ScanContextManger::scanContextMatShift(Eigen::MatrixXf& scan_context, int shift) {
    /**
     * The number of `shift` means moving the columns to the right
    */
    Eigen::MatrixXf shifted_scan_context = Eigen::MatrixXf::Zero(kNumRings, kNumSector);
    if (shift == 0) {
        return scan_context;
    }
    else {
        // Compute the cyclic shift amount
        int cyclic_shift = shift % kNumSector;

        // Copy the rightmost columns to the left of the shifted matrix
        shifted_scan_context.block(0, cyclic_shift, kNumRings, kNumSector - cyclic_shift) = scan_context.block(0, 0, kNumRings, kNumSector - cyclic_shift);

        // Copy the leftmost columns to the right of the shifted matrix
        shifted_scan_context.block(0, 0, kNumRings, cyclic_shift) = scan_context.block(0, kNumSector - cyclic_shift, kNumRings, cyclic_shift);
    }
    return shifted_scan_context;
}

double ScanContextManger::distDirectSC(Eigen::MatrixXf& q_scan_context, Eigen::MatrixXf& ref_scan_context) {
    /**
     * Note: Directly copy from the original code (Not tested yet)
    */
    int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
    double sum_sector_similarity = 0;
    for ( int col_idx = 0; col_idx < q_scan_context.cols(); col_idx++ )
    {
        Eigen::VectorXf col_q_scan_context = q_scan_context.col(col_idx);
        Eigen::VectorXf col_ref_scan_context = ref_scan_context.col(col_idx);

        if( col_q_scan_context.norm() == 0 | col_ref_scan_context.norm() == 0 )
            continue; // don't count this sector pair.

        double sector_similarity = col_q_scan_context.dot(col_ref_scan_context) / (col_q_scan_context.norm() * col_ref_scan_context.norm());

        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        num_eff_cols = num_eff_cols + 1;
    }

    double sc_sim = sum_sector_similarity / num_eff_cols;
    return 1.0 - sc_sim;
}

std::pair<double, int> ScanContextManger::scanContextAlignment(Eigen::MatrixXf& q_scan_context, Eigen::MatrixXf& ref_scan_context) {
    /**
     * compute the similarity score between two scan contexts
     * The return value is a pair of (score, shift), `shift` can convert to angles
    */
    double min_dist = std::numeric_limits<double>::infinity();
    int best_shift = 0;
    for (unsigned int i = 0; i < q_scan_context.cols(); i++) { /* `i==0` equals `i == q_scan_context.cols()`: 0 == 360 */
        /* TODO: It seems that the original code has some faster implemention */
        Eigen::MatrixXf shifted_q_scan_context = scanContextMatShift(q_scan_context, i);
        double cur_sc_dist = distDirectSC( shifted_q_scan_context, ref_scan_context );
        if (cur_sc_dist < min_dist) {
            min_dist = cur_sc_dist;
            best_shift = i;
        }
    }
    return std::make_pair(min_dist, best_shift);
}

std::pair<int, double> ScanContextManger::detectLoopClosure(const POINT_CLOUD_DSC_& cur_dsc) {
    int loop_id = -1;                           // -1 means no loop closure detected
    double loop_angle = 0.0;                    // the angle between the current frame and the loop frame
    // use the ring key to query quickly
    std::vector<float> q_ring_key = cur_dsc.ring_key;
    auto q_scan_context = cur_dsc.scan_context;

    if (point_cloud_dsc_buffer_.size() < kSkipLaestFrames) {
        RCLCPP_INFO(rclcpp::get_logger("ScanContextManger"), "The buffer is not large enough to detect loop closure, the buffer size is %d", point_cloud_dsc_buffer_.size());
        return std::make_pair(loop_id, loop_angle);
    }
    // every `kSkipLaestFrames` frames, reconstruct the ring key tree
    if (point_cloud_dsc_buffer_.size() % kSkipLaestFrames == 0) {
        // RCLCPP_INFO(rclcpp::get_logger("ScanContextManger"), "Reconstruct the ring key tree.");
        tmp_ring_key_search_base_.clear();
        tmp_ring_key_search_base_.assign(ring_key_buffer_.begin(), ring_key_buffer_.end() - kSkipLaestFrames + 1);  //avoid to search the latest frames, in addition, plus 1

        ring_key_kd_tree_.reset();
        ring_key_kd_tree_ = std::make_unique<RingKeyKDTree>(kNumRings /* dim */, tmp_ring_key_search_base_, 10 /* max leaf */ );
    }

    std::vector<size_t> candidate_indexes( kNumRingKeyQuery );
    std::vector<float> out_dists_sqr( kNumRingKeyQuery );

    nanoflann::KNNResultSet<float> knnsearch_result( kNumRingKeyQuery );
    knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
    ring_key_kd_tree_->index->findNeighbors( knnsearch_result, &q_ring_key[0] /* query */, nanoflann::SearchParams(10) );


    double min_dist = std::numeric_limits<double>::infinity(); // init with positive infinity
    int nn_shift = 0;
    int nn_idx = 0;
    // Using the found ring key to refine by searching the scan context
    // record the time
    auto start = std::chrono::high_resolution_clock::now();
    for (unsigned int i = 0; i < kNumRingKeyQuery; i++) {
        Eigen::MatrixXf ref_scan_context = point_cloud_dsc_buffer_[candidate_indexes[i]].scan_context;
        std::pair<double, int> dist_align = scanContextAlignment(q_scan_context, ref_scan_context);
        double cur_dist = dist_align.first;
        int cur_shift = dist_align.second;
        if (cur_dist < min_dist) {
            min_dist = cur_dist;
            nn_shift = cur_shift;
            nn_idx = candidate_indexes[i];
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start); // in ms
    // RCLCPP_INFO(rclcpp::get_logger("ScanContextManger"), "The time of searching the scan context is %d ms", duration_ms.count());

    if (min_dist < kSC_DIST_THRES) {
        loop_id = point_cloud_dsc_buffer_[nn_idx].id;
        loop_angle = deg2rad(nn_shift * kSectorResolution); // 0-2pi
        // RCLCPP_INFO(rclcpp::get_logger("ScanContextManger"), "Loop closure detected, the loop id is %d, the shift is %d", loop_id, nn_shift);
    }
    return std::make_pair(loop_id, loop_angle);
}

std::pair<int, double> ScanContextManger::addNewFrame(const unsigned int id, const pcl::PointCloud<pcl::PointXY>::Ptr cloud) {
    // Add the new frame to the buffer
        POINT_CLOUD_DSC_ pt_dsc = {0};
        pt_dsc.id = id;
        pt_dsc.scan_context = generateScanContext(cloud);
        pt_dsc.ring_key = generateRingKey(pt_dsc.scan_context);
        ring_key_buffer_.push_back(pt_dsc.ring_key);
        point_cloud_dsc_buffer_.push_back(pt_dsc);

    // Use the latest added frame to detect the loop closure
    std::pair<int, double> res = detectLoopClosure(pt_dsc);
    return res;
}

bool ScanContextManger::isPossibleLoop(const MatrixSE2& pose1, const MatrixSE2& pose2) {
    float p1x = pose1(0, 2);
    float p1y = pose1(1, 2);
    float p2x = pose2(0, 2);
    float p2y = pose2(1, 2);
    float dist = sqrt( (p1x - p2x) * (p1x - p2x) + (p1y - p2y) * (p1y - p2y) );
    if (dist < kLoopDistThres) {
        return true;
    } else {
        return false;
    }
}

