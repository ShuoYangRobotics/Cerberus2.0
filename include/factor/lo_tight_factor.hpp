#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

#include "factor/lo_tight_integration_base.hpp"
#include "utils/vins_utility.h"

/*
 * LOTightFactor get raw leg sensor data and generate one factor for leg.
 * It is considered as ``tightly coupled'' because all sensor data are used together.
 */

// size: 7 is pose (position+quaternion)
//       9 is speedBias(velocity + gyroBias + accelBias)
//       7 is footBias [(footIMUbias footVel + kinematic parameter)]

// LO_TIGHT_RESIDUAL_SIZE 16
//  0:2    rotation
//  3:5    Q^T(Pj-Pi)-epsilon
//  6:8    Bgj-Bgi
//  9:11   Bfj-Bfi
// 12:14   Bvj-Bvi
//  15     rhoj-rhoi

class LOTightFactor : public ceres::SizedCostFunction<LO_TIGHT_RESIDUAL_SIZE, 7, 9, 7, 7, 9, 7> {
 public:
  LOTightFactor() = delete;
  LOTightFactor(LOTightIntegrationBase* _lo_pre_integration) : lo_pre_integration(_lo_pre_integration) {}
  virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    // variables at time i
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

    Eigen::Vector3d Bfi(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Vector3d Bvi(parameters[2][3], parameters[2][4], parameters[2][5]);
    Vec_rho rhoi(parameters[2][6]);

    // variables at time j
    Eigen::Vector3d Pj(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Quaterniond Qj(parameters[3][6], parameters[3][3], parameters[3][4], parameters[3][5]);

    Eigen::Vector3d Vj(parameters[4][0], parameters[4][1], parameters[4][2]);
    Eigen::Vector3d Baj(parameters[4][3], parameters[4][4], parameters[4][5]);
    Eigen::Vector3d Bgj(parameters[4][6], parameters[4][7], parameters[4][8]);

    Eigen::Vector3d Bfj(parameters[5][0], parameters[5][1], parameters[5][2]);
    Eigen::Vector3d Bvj(parameters[5][3], parameters[5][4], parameters[5][5]);
    Vec_rho rhoj(parameters[5][6]);

    // get residual
    Eigen::Map<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, 1>> residual(residuals);
    residual = lo_pre_integration->evaluate(Pi, Qi, Bgi, Bfi, Bvi, rhoi, Pj, Qj, Bgj, Bfj, Bvj, rhoj);
    Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE> sqrt_info =
        Eigen::LLT<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE>>(lo_pre_integration->covariance.inverse())
            .matrixL()
            .transpose();
    //        sqrt_info.setIdentity();
    residual = sqrt_info * residual;

    if (jacobians) {
      Eigen::Matrix3d dq_dbg = lo_pre_integration->jacobian.block<3, 3>(T_R, T_BG);
      Eigen::Matrix3d de_dbg = lo_pre_integration->jacobian.block<3, 3>(T_E, T_BG);
      Eigen::Matrix3d de_dbf = lo_pre_integration->jacobian.block<3, 3>(T_E, T_BF);
      Eigen::Matrix3d de_dbv = lo_pre_integration->jacobian.block<3, 3>(T_E, T_BV);
      Eigen::Vector3d de_dbd0 = lo_pre_integration->jacobian.block<3, 1>(T_E, T_RHO);

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
        jacobian_pose_i.setZero();
        Eigen::Quaterniond corrected_delta_q =
            lo_pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - lo_pre_integration->linearized_bg));

        jacobian_pose_i.block<3, 3>(0, 3) =
            -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();

        jacobian_pose_i.block<3, 3>(3, 0) = -Qi.inverse().toRotationMatrix();
        jacobian_pose_i.block<3, 3>(3, 3) = Utility::skewSymmetric(Qi.inverse() * (Pj - Pi));

        jacobian_pose_i = sqrt_info * jacobian_pose_i;
      }

      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
        jacobian_speedbias_i.setZero();
        Eigen::Quaterniond corrected_delta_q =
            lo_pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - lo_pre_integration->linearized_bg));

        jacobian_speedbias_i.block<3, 3>(0, 6) = -Utility::Qleft(Qj.inverse() * Qi * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;

        jacobian_speedbias_i.block<3, 3>(3, 6) = -de_dbg;
        jacobian_speedbias_i.block<3, 3>(6, 6) = -Eigen::Matrix3d::Identity();

        jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;
      }

      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, 7, Eigen::RowMajor>> jacobian_footbias_i(jacobians[2]);
        jacobian_footbias_i.setZero();
        jacobian_footbias_i.block<3, 3>(3, 0) = -de_dbf;
        jacobian_footbias_i.block<3, 3>(3, 3) = -de_dbv;
        jacobian_footbias_i.block<3, 1>(3, 6) = -de_dbd0;

        jacobian_footbias_i.block<3, 3>(9, 0) = -Eigen::Matrix3d::Identity();
        jacobian_footbias_i.block<3, 3>(12, 3) = -Eigen::Matrix3d::Identity();
        jacobian_footbias_i(15, 6) = -1;  // rho

        jacobian_footbias_i = sqrt_info * jacobian_footbias_i;
      }

      if (jacobians[3]) {
        Eigen::Map<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[3]);
        jacobian_pose_j.setZero();
        Eigen::Quaterniond corrected_delta_q =
            lo_pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - lo_pre_integration->linearized_bg));
        jacobian_pose_j.block<3, 3>(0, 3) = Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
        jacobian_pose_j.block<3, 3>(3, 0) = Qi.inverse().toRotationMatrix();

        jacobian_pose_j = sqrt_info * jacobian_pose_j;
      }

      if (jacobians[4]) {
        Eigen::Map<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[4]);
        jacobian_speedbias_j.setZero();
        jacobian_speedbias_j.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
        jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;
      }

      if (jacobians[5]) {
        Eigen::Map<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, 7, Eigen::RowMajor>> jacobian_footbias_j(jacobians[5]);
        jacobian_footbias_j.setZero();

        jacobian_footbias_j.block<3, 3>(9, 0) = Eigen::Matrix3d::Identity();
        jacobian_footbias_j.block<3, 3>(12, 3) = Eigen::Matrix3d::Identity();
        jacobian_footbias_j(15, 6) = 1;  // rho

        jacobian_footbias_j = sqrt_info * jacobian_footbias_j;
      }
    }
    return true;
  }

  void checkJacobian(const double* const* parameters) {
    // variables at time i
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

    Eigen::Vector3d Bfi(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Vector3d Bvi(parameters[2][3], parameters[2][4], parameters[2][5]);
    Vec_rho rhoi(parameters[2][6]);

    // variables at time j
    Eigen::Vector3d Pj(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Quaterniond Qj(parameters[3][6], parameters[3][3], parameters[3][4], parameters[3][5]);

    Eigen::Vector3d Vj(parameters[4][0], parameters[4][1], parameters[4][2]);
    Eigen::Vector3d Baj(parameters[4][3], parameters[4][4], parameters[4][5]);
    Eigen::Vector3d Bgj(parameters[4][6], parameters[4][7], parameters[4][8]);

    Eigen::Vector3d Bfj(parameters[5][0], parameters[5][1], parameters[5][2]);
    Eigen::Vector3d Bvj(parameters[5][3], parameters[5][4], parameters[5][5]);
    Vec_rho rhoj(parameters[5][6]);

    // evaluate once

    std::vector<int> block_sizes{7, 9, 7, 7, 9, 7};
    Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, 1> residual;
    double** raw_jacobians = new double*[block_sizes.size()];
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
    jacobians.resize(block_sizes.size());
    for (int xx = 0; xx < static_cast<int>(block_sizes.size()); xx++) {
      jacobians[xx].resize(LO_TIGHT_RESIDUAL_SIZE, block_sizes[xx]);
      raw_jacobians[xx] = jacobians[xx].data();
      // dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
    }

    Evaluate(parameters, residual.data(), raw_jacobians);

    // perturb Pi
    Eigen::Vector3d turb(0.0001, -0.003, 0.003);
    Eigen::Matrix<double, 7, 1> turb_vec;
    turb_vec.setZero();
    turb_vec.segment<3>(0) = turb;
    Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, 1> turb_residual =
        lo_pre_integration->evaluate(Pi + turb, Qi, Bgi, Bfi, Bvi, rhoi, Pj, Qj, Bgj, Bfj, Bvj, rhoj);
    Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE> sqrt_info2 =
        Eigen::LLT<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE>>(lo_pre_integration->covariance.inverse())
            .matrixL()
            .transpose();
    //    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    Eigen::VectorXd tmp = turb_residual - (jacobians[0] * turb_vec + residual);
    // std::cout << "residual\t" << residual.transpose() << std::endl;
    // std::cout << "turb_residual\t" << turb_residual.transpose() << std::endl;
    // std::cout << "jac_residual\t" << (jacobians[0] * turb_vec + residual).transpose() << std::endl;
    std::cout << "perturb Pi\t" << tmp.maxCoeff() << std::endl;

    // perturb Qi
    turb_residual = lo_pre_integration->evaluate(Pi, Qi * Eigen::Quaterniond(1, turb(0) / 2, turb(1) / 2, turb(2) / 2).normalized(), Bgi,
                                                 Bfi, Bvi, rhoi, Pj, Qj, Bgj, Bfj, Bvj, rhoj);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE>>(lo_pre_integration->covariance.inverse())
                     .matrixL()
                     .transpose();
    //    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    turb_vec.setZero();
    turb_vec.segment<3>(3) = turb;

    tmp = turb_residual - (jacobians[0] * turb_vec + residual);
    // std::cout << "residual\t" << residual.transpose() << std::endl;
    // std::cout << "turb_residual\t" << turb_residual.transpose() << std::endl;
    // std::cout << "jac_residual\t" << (jacobians[0] * turb_vec + residual).transpose() << std::endl;
    std::cout << "perturb Qi\t" << tmp.maxCoeff() << std::endl;

    // perturb Bgi
    Eigen::Matrix<double, 9, 1> turb_vec9;
    turb_vec9.setZero();
    turb_vec9.segment<3>(6) = turb;

    turb_residual = lo_pre_integration->evaluate(Pi, Qi, Bgi + turb, Bfi, Bvi, rhoi, Pj, Qj, Bgj, Bfj, Bvj, rhoj);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE>>(lo_pre_integration->covariance.inverse())
                     .matrixL()
                     .transpose();
    //    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    tmp = turb_residual - (jacobians[1] * turb_vec9 + residual);
    std::cout << "perturb Bgi\t" << tmp.maxCoeff() << std::endl;

    // perturb Bfi
    Eigen::Matrix<double, 7, 1> foot_bias_turb_vec;
    foot_bias_turb_vec.setZero();
    foot_bias_turb_vec.segment<3>(0) = turb;

    turb_residual = lo_pre_integration->evaluate(Pi, Qi, Bgi, Bfi + turb, Bvi, rhoi, Pj, Qj, Bgj, Bfj, Bvj, rhoj);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE>>(lo_pre_integration->covariance.inverse())
                     .matrixL()
                     .transpose();
    //    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    tmp = turb_residual - (jacobians[2] * foot_bias_turb_vec + residual);
    std::cout << "perturb Bfi\t" << tmp.maxCoeff() << std::endl;

    // perturb Bvi
    foot_bias_turb_vec.setZero();
    foot_bias_turb_vec.segment<3>(3) = turb;

    turb_residual = lo_pre_integration->evaluate(Pi, Qi, Bgi, Bfi, Bvi + turb, rhoi, Pj, Qj, Bgj, Bfj, Bvj, rhoj);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE>>(lo_pre_integration->covariance.inverse())
                     .matrixL()
                     .transpose();
    //    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    tmp = turb_residual - (jacobians[2] * foot_bias_turb_vec + residual);
    std::cout << "perturb Bvi\t" << tmp.maxCoeff() << std::endl;

    // perturb rhoi
    foot_bias_turb_vec.setZero();
    foot_bias_turb_vec(6) = turb(0);
    Vec_rho turb_rho = rhoi;
    turb_rho(0) += turb(0);
    turb_residual = lo_pre_integration->evaluate(Pi, Qi, Bgi, Bfi, Bvi, turb_rho, Pj, Qj, Bgj, Bfj, Bvj, rhoj);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE>>(lo_pre_integration->covariance.inverse())
                     .matrixL()
                     .transpose();
    //    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    tmp = turb_residual - (jacobians[2] * foot_bias_turb_vec + residual);
    std::cout << "perturb Rho_i\t" << tmp.maxCoeff() << std::endl;

    // perturb Pj
    turb_vec.setZero();
    turb_vec.segment<3>(0) = turb;
    turb_residual = lo_pre_integration->evaluate(Pi, Qi, Bgi, Bfi, Bvi, rhoi, Pj + turb, Qj, Bgj, Bfj, Bvj, rhoj);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE>>(lo_pre_integration->covariance.inverse())
                     .matrixL()
                     .transpose();
    //    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    tmp = turb_residual - (jacobians[3] * turb_vec + residual);
    std::cout << "perturb Pj\t" << tmp.maxCoeff() << std::endl;

    // perturb Qj
    turb_residual =
        lo_pre_integration->evaluate(Pi, Qi, Bgi, Bfi, Bvi, rhoi, Pj,
                                     Qj * Eigen::Quaterniond(1, turb(0) / 2, turb(1) / 2, turb(2) / 2).normalized(), Bgj, Bfj, Bvj, rhoj);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE>>(lo_pre_integration->covariance.inverse())
                     .matrixL()
                     .transpose();
    //    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    turb_vec.setZero();
    turb_vec.segment<3>(3) = turb;

    tmp = turb_residual - (jacobians[3] * turb_vec + residual);
    std::cout << "perturb Qj\t" << tmp.maxCoeff() << std::endl;

    // perturb Bgj
    turb_vec9.setZero();
    turb_vec9.segment<3>(6) = turb;

    turb_residual = lo_pre_integration->evaluate(Pi, Qi, Bgi, Bfi, Bvi, rhoi, Pj, Qj, Bgj + turb, Bfj, Bvj, rhoj);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE>>(lo_pre_integration->covariance.inverse())
                     .matrixL()
                     .transpose();
    //    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    tmp = turb_residual - (jacobians[4] * turb_vec9 + residual);
    std::cout << "perturb Bgj\t" << tmp.maxCoeff() << std::endl;

    // perturb Bfj
    foot_bias_turb_vec.setZero();
    foot_bias_turb_vec.segment<3>(0) = turb;

    turb_residual = lo_pre_integration->evaluate(Pi, Qi, Bgi, Bfi, Bvi, rhoi, Pj, Qj, Bgj, Bfj + turb, Bvj, rhoj);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE>>(lo_pre_integration->covariance.inverse())
                     .matrixL()
                     .transpose();
    //    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    tmp = turb_residual - (jacobians[5] * foot_bias_turb_vec + residual);
    std::cout << "perturb Bfj\t" << tmp.maxCoeff() << std::endl;

    // perturb Bvj
    foot_bias_turb_vec.setZero();
    foot_bias_turb_vec.segment<3>(3) = turb;

    turb_residual = lo_pre_integration->evaluate(Pi, Qi, Bgi, Bfi, Bvi, rhoi, Pj, Qj, Bgj, Bfj, Bvj + turb, rhoj);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE>>(lo_pre_integration->covariance.inverse())
                     .matrixL()
                     .transpose();
    //    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    tmp = turb_residual - (jacobians[5] * foot_bias_turb_vec + residual);
    std::cout << "perturb Bvj\t" << tmp.maxCoeff() << std::endl;

    // perturb rhoj
    foot_bias_turb_vec.setZero();
    foot_bias_turb_vec(6) = turb(0);
    turb_rho = rhoj;
    turb_rho(0) += turb(0);
    turb_residual = lo_pre_integration->evaluate(Pi, Qi, Bgi, Bfi, Bvi, rhoi, Pj, Qj, Bgj, Bfj, Bvj, turb_rho);
    sqrt_info2 = Eigen::LLT<Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE>>(lo_pre_integration->covariance.inverse())
                     .matrixL()
                     .transpose();
    //    sqrt_info2.setIdentity();
    turb_residual = sqrt_info2 * turb_residual;

    tmp = turb_residual - (jacobians[5] * foot_bias_turb_vec + residual);
    std::cout << "perturb Rho_j\t" << tmp.maxCoeff() << std::endl;
  }

  LOTightIntegrationBase* lo_pre_integration;
};