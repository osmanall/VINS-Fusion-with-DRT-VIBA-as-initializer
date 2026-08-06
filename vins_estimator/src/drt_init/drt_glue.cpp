#include "drt_glue.h"
#include "initMethod/drtLooselyCoupled.h"
#include "IMU/imuPreintegrated.hpp"
#include "IMU/basicTypes.hpp"
#include <cmath>

DrtInitOut RunDrtInit(
    const Eigen::Matrix3d& Ric, const Eigen::Vector3d& Tic,
    double gyr_n, double acc_n, double gyr_w, double acc_w,
    const std::vector<double>& stamps,
    const std::vector<std::map<int, std::vector<std::pair<int, Eigen::Matrix<double,7,1>>>>>& feats,
    const std::vector<std::vector<DrtImuSample>>& imus)
{
    DrtInitOut out;
    DRT::drtLooselyCoupled drt(Ric, Tic);
    int N = (int)stamps.size();
    for (int i = 0; i < N; i++) {
        FeatureTrackerResulst image;
        for (auto& f : feats[i])
            for (auto& obs : f.second)
                image[f.first].emplace_back(obs.first, obs.second);
        drt.addFeatureCheckParallax(stamps[i], image, 0.0);
        if (i > 0) {
            const auto& samples = imus[i-1];
            double dt0 = samples.empty() ? 0.005 : samples[0].dt;
            double sf = std::sqrt(1.0 / std::max(dt0, 1e-4));
            vio::IMUBias bias;
            vio::IMUCalibParam calib(Ric, Tic, gyr_n*sf, acc_n*sf, gyr_w/sf, acc_w/sf);
            vio::IMUPreintegrated pre(bias, &calib, stamps[i-1], stamps[i]);
            for (auto& s : samples)
                if (s.dt > 0) pre.integrate_new_measurement(s.gyr, s.acc, s.dt);
            drt.addImuMeasure(pre);
        }
    }
    if (!drt.process()) return out;
    if ((int)drt.rotation.size() != N) return out;
    out.R = drt.rotation; out.P = drt.position; out.V = drt.velocity;
    out.bg = drt.biasg;   out.ba = drt.biasa;   out.g = drt.gravity;
    out.ok = true;
    return out;
}