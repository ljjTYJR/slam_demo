// test for libpointmatcher

#include <iostream>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/dynamic_autodiff_cost_function.h>
#include <Eigen/Eigen>

#include "pointmatcher/PointMatcher.h"

using namespace std;
int main() {
    // generate 100 ramdom 2d points
    Eigen::MatrixXd points(2, 100);
    for (int i = 0; i < 100; i++) {
        points(0, i) = rand() % 100;
        points(1, i) = rand() % 100;
    }
    // generate a random small angle
    double angle = rand() % 100 * 0.01;
    // create a rotation matrix
    Eigen::Matrix2d R;
    R << cos(angle), -sin(angle), sin(angle), cos(angle);
    cout << "R = " << endl << R << endl;

    Eigen::Vector2d t(0.01, 0.02);
    cout << "t = " << endl << t << endl;
    // Transform the points
    Eigen::MatrixXd points_transformed = R * points + t.replicate(1, 100);
    // Add noise

    // use libpointmatcher to compute the transformation
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    DP points_dp;
    DP points_transformed_dp;
    points_dp.features.resize(3, 100);
    points_transformed_dp.features.resize(3, 100);
    for (int i = 0; i < 100; i++) {
        points_dp.features(0, i) = points(0, i);
        points_dp.features(1, i) = points(1, i);
        points_dp.features(2, i) = 1;
        points_transformed_dp.features(0, i) = points_transformed(0, i);
        points_transformed_dp.features(1, i) = points_transformed(1, i);
        points_transformed_dp.features(2, i) = 1;
    }
    // add initial guess
    PM::TransformationParameters T_init = PM::TransformationParameters::Identity(3, 3);
    T_init(0, 2) = 0.01;
    T_init(1, 2) = 0.02;
    T_init(0, 0) = cos(angle);
    T_init(0, 1) = -sin(angle);
    T_init(1, 0) = sin(angle);
    T_init(1, 1) = cos(angle);


    PM::ICP icp;
    PointMatcherSupport::Parametrizable::Parameters params;
    std::string name;
    /* set the params */
    name = "KDTreeMatcher";
    params["knn"] = "1";
    std::shared_ptr<PM::Matcher> kdtree = PM::get().MatcherRegistrar.create(name, params);
    params.clear();
    name = "TrimmedDistOutlierFilter";
    params["ratio"] = "1";
    std::shared_ptr<PM::OutlierFilter> trim = PM::get().OutlierFilterRegistrar.create(name, params);
    params.clear();
    name = "PointToPointErrorMinimizer";
    std::shared_ptr<PM::ErrorMinimizer> pointToPoint = PM::get().ErrorMinimizerRegistrar.create(name);
    name = "CounterTransformationChecker";
    params["maxIterationCount"] = "50";
    std::shared_ptr<PM::TransformationChecker> maxIter = PM::get().TransformationCheckerRegistrar.create(name, params);
    params.clear();
    std::shared_ptr<PM::Transformation> rigidTrans = PM::get().TransformationRegistrar.create("RigidTransformation");
    std::shared_ptr<PM::Inspector> nullInspect = PM::get().InspectorRegistrar.create("NullInspector");
    icp.matcher = kdtree;
    icp.outlierFilters.push_back(trim);
    icp.errorMinimizer = pointToPoint;
    icp.transformationCheckers.push_back(maxIter);
    icp.transformations.push_back(rigidTrans);
    icp.inspector = nullInspect;

    PM::TransformationParameters T = icp(points_transformed_dp, points_dp, T_init);
    cout << "T = " << endl << T << endl;
    PM::TransformationParameters T2 = icp(points_dp, points_transformed_dp, T_init);
    cout << "T2 = " << endl << T2 << endl;


    return 0;
}