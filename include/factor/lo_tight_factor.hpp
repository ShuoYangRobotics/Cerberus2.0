#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

#include "factor/lo_tight_intergration_base.hpp"
#include "utils/vins_utility.h"

/*
 * LOTightFactor get raw leg sensor data and generate one factor for leg.
 * It is considered as ``tightly coupled'' because all sensor data are used together.
 */

// size: 7 is pose (position+quaternion)
//       9 is speedBias(velocity + gyroBias + accelBias)
//       4 is footBias(footIMUbias + kinematic parameter)

class LOTightFactor : public ceres::SizedCostFunction<LO_TIGHT_RESIDUAL_SIZE, 7, 9, 4, 7, 9, 4> {
 public:
  IMUFactor() = delete;
  IMUFactor(IntegrationBase* _pre_integration) : pre_integration(_pre_integration) {}
  virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    // variables at time i
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

    Eigen::Vector3d Bfi(parameters[2][0], parameters[2][1], parameters[2][2]);
    Vec_rho rhoi(parameters[2][3]);

    // variables at time j
    Eigen::Vector3d Pj(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Quaterniond Qj(parameters[3][6], parameters[3][3], parameters[3][4], parameters[3][5]);

    Eigen::Vector3d Vj(parameters[4][0], parameters[4][1], parameters[4][2]);
    Eigen::Vector3d Baj(parameters[4][3], parameters[4][4], parameters[4][5]);
    Eigen::Vector3d Bgj(parameters[4][6], parameters[4][7], parameters[4][8]);

    Eigen::Vector3d Bfj(parameters[5][0], parameters[5][1], parameters[5][2]);
    Vec_rho rhoj(parameters[5][3]);

    // get residual
    Eigen::Map<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, 1>> residual(residuals);
    residual = lo_pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi, Bfi, rhoi, Pj, Qj, Vj, Baj, Bgj, Bfj, rhoj);
    Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE> sqrt_info =
        Eigen::LLT<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE>>(lo_pre_integration->covariance.inverse())
            .matrixL()
            .transpose();
    //        sqrt_info.setIdentity();
    residual = sqrt_info * residual;

    if (jacobians) {
    }
  }

  LOTightIntegrationBase* lo_pre_integration;
};