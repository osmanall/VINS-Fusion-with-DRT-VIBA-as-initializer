#pragma once
#include <vector>
#include <map>
#include <utility>
#include <Eigen/Dense>

// One raw IMU sample between two frames.
struct DrtImuSample { Eigen::Vector3d gyr, acc; double dt; };

// Plain-typed bridge to DRT+VI-BA (keeps DRT headers out of estimator.cpp).
struct DrtInitOut {
    bool ok = false;
    std::vector<Eigen::Matrix3d> R;          // body-in-world per frame
    std::vector<Eigen::Vector3d> P, V;
    Eigen::Vector3d bg, ba, g;
};

DrtInitOut RunDrtInit(
    const Eigen::Matrix3d& Ric, const Eigen::Vector3d& Tic,
    double gyr_n, double acc_n, double gyr_w, double acc_w,
    const std::vector<double>& stamps,
    const std::vector<std::map<int, std::vector<std::pair<int, Eigen::Matrix<double,7,1>>>>>& feats,
    const std::vector<std::vector<DrtImuSample>>& imus);