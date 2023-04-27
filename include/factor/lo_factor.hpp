#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

#include "factor/lo_intergration_base.hpp"
#include "utils/vins_utility.h"

/*
 * LOFactor directly intergrates the velocity measurements from PO as a displacement measurement for the pose.
 * It is considered as ``loosely coupled'' because raw sensor data is first processed by PO, and then feed into VILO.
 */

class LOFactor : public ceres::SizedCostFunction<LO_RESIDUAL_SIZE, 7, 7> {
 public:
  LOFactor() = delete;
  LOFactor(LOIntegrationBase* _lo_pre_integration) { lo_pre_integration = _lo_pre_integration; }

  // para_Pose[i], para_Pose[j]
  virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);

    Eigen::Map<Eigen::Matrix<double, LO_RESIDUAL_SIZE, 1>> residual(residuals);
    residual = lo_pre_integration->evaluate(Pi, Pj);

    Eigen::Matrix<double, LO_RESIDUAL_SIZE, LO_RESIDUAL_SIZE> sqrt_info =
        Eigen::LLT<Eigen::Matrix<double, LO_RESIDUAL_SIZE, LO_RESIDUAL_SIZE>>(lo_pre_integration->covariance.inverse())
            .matrixL()
            .transpose();
    // sqrt_info.setIdentity();
    residual = sqrt_info * residual;
    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, LO_RESIDUAL_SIZE, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

        jacobian_pose_i.setZero();
        jacobian_pose_i.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
        jacobian_pose_i = sqrt_info * jacobian_pose_i;
      }

      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, LO_RESIDUAL_SIZE, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

        jacobian_pose_j.setZero();
        jacobian_pose_j.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        jacobian_pose_j = sqrt_info * jacobian_pose_j;
      }
    }

    return true;
  }

  LOIntegrationBase* lo_pre_integration;
};