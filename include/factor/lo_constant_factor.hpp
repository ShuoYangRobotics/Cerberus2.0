#pragma once
#include <ceres/ceres.h>
#include <Eigen/Dense>

#include "factor/lo_tight_integration_base.hpp"  // just use Vec_rho
#include "utils/vins_utility.h"

// leg id?
// if contact preintegration is not possible (foot is not in contact between two camera frames).
// we add this LOConstantFactor, where the footBiases are kept constant
class LOConstantFactor : public ceres::SizedCostFunction<7, 7, 7> {
 public:
  LOConstantFactor() { leg_id = 0; }
  LOConstantFactor(int _leg_id) { leg_id = _leg_id; }

  // para_footBias[leg_id][i], para_footBias[leg_id][j]
  virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    Eigen::Vector3d Bfi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Vector3d Bvi(parameters[0][3], parameters[0][4], parameters[0][5]);
    Vec_rho rhoi(parameters[0][6]);

    Eigen::Vector3d Bfj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Bvj(parameters[1][3], parameters[1][4], parameters[1][5]);
    Vec_rho rhoj(parameters[1][6]);

    Eigen::Map<Eigen::Matrix<double, 7, 1>> residual(residuals);
    residual.segment<3>(0) = Bfj - Bfi;
    residual.segment<3>(3) = Bvj - Bvi;
    residual.segment<1>(6) = rhoj - rhoi;

    Eigen::Matrix<double, 7, 7> sqrt_info;
    sqrt_info.setIdentity();  // change to some value?
    residual = sqrt_info * residual;
    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> jacobian_footBias_i(jacobians[0]);
        jacobian_footBias_i.setZero();
        jacobian_footBias_i.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
        jacobian_footBias_i.block<3, 3>(3, 3) = -Eigen::Matrix3d::Identity();
        jacobian_footBias_i(6, 6) = -1;
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> jacobian_footBias_j(jacobians[1]);
        jacobian_footBias_j.setZero();
        jacobian_footBias_j.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        jacobian_footBias_j.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
        jacobian_footBias_j(6, 6) = 1;
      }
    }
    return true;
  }

 private:
  int leg_id;
};