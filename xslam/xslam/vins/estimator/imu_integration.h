

#include <ceres/ceres.h>

#ifndef XSLAM_VINS_ESTIMATOR_IMU_INTEGRATION_H
#define XSLAM_VINS_ESTIMATOR_IMU_INTEGRATION_H

#include <thread>
#include <string>
#include <deque>

#include "xslam/vins/sensor/imu_data.h"
#include "xslam/vins/common/time.h"
#include "xslam/vins/estimator/proto/imu_noise.pb.h"

namespace xslam {
namespace vins {
namespace estimator {
    
enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

// PVQ ba bg
struct ImuPose
{
    Eigen::Vector3d position;
    Eigen::Quaterniond quaternion; 
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration_bias;
    Eigen::Vector3d gyro_bias;
};

// PVQ delta
struct DeltaPose
{
    Eigen::Vector3d position;
    Eigen::Quaterniond quaternion;
    Eigen::Vector3d velocity;

    void Reset()
    {
        position.setZero();
        quaternion.setIdentity();
        velocity.setZero();
    }
};

// X = F * X + G * n
class ImuIntegration 
{
public:
    using States = Eigen::Matrix<double, 15, 1>;
    using Jacobian = Eigen::Matrix<double, 15, 15>;
    using Covariance = Eigen::Matrix<double, 15, 15>;
    using VStates = Eigen::Matrix<double, 15, 18>;
    using Noise = Eigen::Matrix<double, 18, 18>;


    explicit ImuIntegration(const sensor::ImuData& data);
    virtual ~ImuIntegration() {}

    ImuIntegration(const ImuIntegration& other) = delete;
    ImuIntegration& operator=(const ImuIntegration& ohter) = delete;

    void Repropagate(
        const Eigen::Vector3d& acceleration_linearized_bias, 
        const Eigen::Vector3d& gyro_linearized_bias);

    void Propagate(const sensor::ImuData& data);

    void AddImuData(const sensor::ImuData& data);

    void MidPointIntegration(double _dt, 
                            const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                            const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian);

    States Evaluate(const ImuPose& pose_i, const ImuPose& pose_j);

private:
    proto::ImuNoiseOptions options_;
    common::Time time_;
    double sum_dt_;
    sensor::ImuData imu_i; // i 时刻
    sensor::ImuData imu_j; // j 时刻

    Eigen::Vector3d linearized_ba_;
    Eigen::Vector3d linearized_bg_;

    Jacobian jacobian_;
    Covariance covariance_;

    Jacobian step_jacobian_;
    VStates step_V_;
    Noise noise_;

    DeltaPose delta_;
    std::deque<sensor::ImuData> queue_;
};

} // namespace estimator
} // namespace vins
} // namespace xslam 

#endif // XSLAM_VINS_ESTIMATOR_IMU_INTEGRATION_H