#pragma once

#include <Eigen/Dense>

#define LO_RESIDUAL_SIZE 3
#define LO_NOISE_SIZE 6

class LOIntegrationBase {
 public:
  LOIntegrationBase() = delete;

  LOIntegrationBase(const Eigen::Vector3d& _vel_0, const Eigen::Matrix3d& _cov_0) {
    vel_0 = _vel_0;
    cov_0 = _cov_0;
    sum_dt = 0.0;
    delta_p.setZero();
    jacobian.setIdentity();
    covariance.setZero();

    // set noise
    noise.setZero();
    noise.block<3, 3>(0, 0) = _cov_0;
    noise(3, 3) = 0.01;
    noise(4, 4) = 0.01;
    noise(5, 5) = 0.1;
  }

  void push_back(double dt, const Eigen::Vector3d& vel, const Eigen::Matrix3d& cov) {
    dt_buf.push_back(dt);
    vel_buf.push_back(vel);

    propagate(dt, vel, cov);
  }

  // do not need to repropagate if no velocity bias is involved

  void propagate(double _dt, const Eigen::Vector3d& _vel_1, const Eigen::Matrix3d& _cov_1) {
    dt = _dt;
    vel_1 = _vel_1;
    cov_1 = _cov_1;
    Vector3d result_delta_p;

    noise.setZero();
    noise.block<3, 3>(0, 0) = cov_0;
    noise(2, 2) *= 100;  // do not use LO z velocity

    noise.block<3, 3>(3, 3) = cov_1;
    noise(5, 5) *= 100;  // do not use LO z velocity

    midPointIntegration(dt, vel_0, vel_1, delta_p, result_delta_p, true);

    delta_p = result_delta_p;

    sum_dt += dt;
    vel_0 = vel_1;
    cov_0 = cov_1;
  }

  Eigen::Matrix<double, LO_RESIDUAL_SIZE, 1> evaluate(const Eigen::Vector3d& Pi, const Eigen::Vector3d& Pj) {
    Eigen::Matrix<double, LO_RESIDUAL_SIZE, 1> residuals;
    residuals = Pj - (Pi + delta_p);
    return residuals;
  }

  void midPointIntegration(double _dt, const Eigen::Vector3d& _vel_0, const Eigen::Vector3d& _vel_1, const Eigen::Vector3d& delta_p,
                           Eigen::Vector3d& result_delta_p, bool update_jacobian) {
    Vector3d average_vel = 0.5 * (_vel_0 + _vel_1);
    result_delta_p = delta_p + average_vel * _dt;

    if (update_jacobian) {
      // trivial jacobian
      Eigen::Matrix<double, LO_RESIDUAL_SIZE, LO_RESIDUAL_SIZE> F;
      F.setIdentity();
      jacobian = F * jacobian;

      Eigen::Matrix<double, LO_RESIDUAL_SIZE, LO_NOISE_SIZE> V;
      V.block<3, 3>(0, 0) = 0.5 * _dt * Eigen::Matrix3d::Identity();
      V.block<3, 3>(0, 3) = 0.5 * _dt * Eigen::Matrix3d::Identity();
      covariance = F * covariance * F.transpose() + V * noise * V.transpose();
    }
  }

  double sum_dt;
  Eigen::Vector3d delta_p;  // integration of lo velocity
  Eigen::Matrix<double, LO_RESIDUAL_SIZE, LO_RESIDUAL_SIZE> jacobian, covariance;

 private:
  double dt;
  Eigen::Vector3d vel_0, vel_1;
  Eigen::Matrix3d cov_0, cov_1;

  std::vector<double> dt_buf;
  std::vector<Eigen::Vector3d> vel_buf;

  Eigen::Matrix<double, LO_NOISE_SIZE, LO_NOISE_SIZE> noise;  // vel0 and vel1
};