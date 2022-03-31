#include "xslam/vins/estimator/imu_integration.h"


namespace xslam {
namespace vins {
namespace estimator {

double ACC_N, ACC_W;
double GYR_N, GYR_W;

ImuIntegration::ImuIntegration(const sensor::ImuData& data)
{
    noise_ = Eigen::Matrix<double, 18, 18>::Zero();
    noise_.block<3, 3>(0, 0) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
    noise_.block<3, 3>(3, 3) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
    noise_.block<3, 3>(6, 6) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
    noise_.block<3, 3>(9, 9) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
    noise_.block<3, 3>(12, 12) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
    noise_.block<3, 3>(15, 15) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();

    imu_i = data;
    jacobian_ = Eigen::Matrix<double, 15, 15>::Identity();
    covariance_ = Eigen::Matrix<double, 15, 15>::Zero();

    delta_.position =  Eigen::Vector3d::Zero();
    delta_.quaternion = Eigen::Quaterniond::Identity();
    delta_.velocity = Eigen::Vector3d::Zero();
}

void ImuIntegration::Repropagate(
    const Eigen::Vector3d& acceleration_linearized_bias, 
    const Eigen::Vector3d& gyro_linearized_bias)
{
      sum_dt_ = 0.0;

      delta_.Reset();
      jacobian_.setIdentity();
      covariance_.setZero();
}

void ImuIntegration::Propagate(const sensor::ImuData& data)
{

      const double delta_t = common::ToSeconds(data.time - time_);
      imu_j = data;

     
        // midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
        //                     linearized_ba, linearized_bg,
        //                     result_delta_p, result_delta_q, result_delta_v,
        //                     result_linearized_ba, result_linearized_bg, 1);

        //checkJacobian(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v,
        //                    linearized_ba, linearized_bg);
        // delta_p = result_delta_p;
        // delta_q = result_delta_q;
        // delta_v = result_delta_v;
        // linearized_ba = result_linearized_ba;
        // linearized_bg = result_linearized_bg;
        // delta_q.normalize();
        // sum_dt += dt;
        // acc_0 = acc_1;
        // gyr_0 = gyr_1;  

        imu_i = imu_j;
}

void ImuIntegration::AddImuData(const sensor::ImuData& data)
{
    queue_.push_back(data);
    Propagate(data);
}

ImuIntegration::States ImuIntegration::Evaluate(const ImuPose& pose_i, const ImuPose& pose_j)
{
    Eigen::Matrix<double, 15, 1> residuals;

    Eigen::Matrix3d dp_dba = jacobian_.block<3, 3>(O_P, O_BA);
    Eigen::Matrix3d dp_dbg = jacobian_.block<3, 3>(O_P, O_BG);

    Eigen::Matrix3d dq_dbg = jacobian_.block<3, 3>(O_R, O_BG);

    Eigen::Matrix3d dv_dba = jacobian_.block<3, 3>(O_V, O_BA);
    Eigen::Matrix3d dv_dbg = jacobian_.block<3, 3>(O_V, O_BG);

    pose_i.acceleration_bias - linearized_ba_;
    Eigen::Vector3d dba =  pose_i.acceleration_bias - linearized_ba_;
    Eigen::Vector3d dbg =  pose_i.gyro_bias - linearized_bg_;

    // Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
    // Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
    // Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

    // residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
    // residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
    // residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
    // residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
    // residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
    return residuals;
}

} // namespace estimator
} // namespace vins
} // namespace xslam 