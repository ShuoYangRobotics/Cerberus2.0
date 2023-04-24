#pragma once
#include <Eigen/Dense>

#include "utils/LOTightUtils.hpp"
#include "utils/parameters.hpp"
#include "utils/vins_utility.h"

#define LO_TIGHT_RESIDUAL_SIZE 16
#define LO_TIGHT_NOISE_SIZE 22
#define RHO_SIZE 1
typedef Eigen::Matrix<double, RHO_SIZE, 1> Vec_rho;

enum T_StateOrder { T_R = 0, T_E = 3, T_BG = 6, T_BF = 9, T_BV = 12, T_RHO = 15 };

enum T_NoiseOrder { T_PHIN = 0, T_DPHIN = 3, T_GN = 6, T_GW = 9, T_FN = 12, T_FW = 15, T_VN = 18, TRHO_N = 21 };

class LOTightIntegrationBase {
 public:
  LOTightIntegrationBase() = delete;

  LOTightIntegrationBase(const int _leg_id, const Eigen::Vector3d& _jang_0, const Eigen::Vector3d& _jvel_0,
                         const Eigen::Vector3d& _body_gyr_0, const Eigen::Vector3d& _foot_gyr_0, const Eigen::Vector3d& _linearized_bg,
                         const Eigen::Vector3d& _linearized_bf, const Eigen::Vector3d& _linearized_bv, const Vec_rho& _linearized_rho,
                         LOTightUtils* _tightUtils)
      : body_gyr_init{_body_gyr_0},
        foot_gyr_init{_foot_gyr_0},
        jang_init{_jang_0},
        jvel_init{_jvel_0},
        linearized_bg{_linearized_bg},
        linearized_bf{_linearized_bf},
        linearized_bv{_linearized_bv},
        linearized_rho{_linearized_rho} {
    leg_id = _leg_id;
    body_gyr_0 = _body_gyr_0;
    foot_gyr_0 = _foot_gyr_0;
    jang_0 = _jang_0;
    jvel_0 = _jvel_0;

    sum_dt = 0.0;
    delta_epsilon.setZero();
    delta_q.setIdentity();
    jacobian.setZero();
    covariance.setZero();

    // init noise
    noise_diag.diagonal() = 1e-2 * Eigen::Matrix<double, LO_TIGHT_NOISE_SIZE, 1>::Ones();
    noise_diag.diagonal().segment<3>(T_PHIN) = JOINT_ANG_N * JOINT_ANG_N * Eigen::Matrix<double, 3, 1>::Ones();
    noise_diag.diagonal().segment<3>(T_DPHIN) = JOINT_VEL_N * JOINT_VEL_N * Eigen::Matrix<double, 3, 1>::Ones();
    noise_diag.diagonal().segment<3>(T_GN) = GYR_N * GYR_N * Eigen::Matrix<double, 3, 1>::Ones();
    noise_diag.diagonal().segment<3>(T_GW) = GYR_W * GYR_W * Eigen::Matrix<double, 3, 1>::Ones();
    noise_diag.diagonal().segment<3>(T_FN) = FOOT_GYR_N * FOOT_GYR_N * Eigen::Matrix<double, 3, 1>::Ones();
    noise_diag.diagonal().segment<3>(T_FW) = FOOT_GYR_W * FOOT_GYR_W * Eigen::Matrix<double, 3, 1>::Ones();
    noise_diag.diagonal().segment<3>(T_VN) = FOOT_VEL_W * FOOT_VEL_W * Eigen::Matrix<double, 3, 1>::Ones();
    noise_diag.diagonal()(TRHO_N) = RHO_W * RHO_W;

    tightUtils = _tightUtils;
  }

  void push_back(const double dt, const Eigen::Vector3d& body_gyr, const Eigen::Vector3d& foot_gyr, const Eigen::Vector3d& jang,
                 const Eigen::Vector3d& jvel) {
    dt_buf.push_back(dt);
    body_gyr_buf.push_back(body_gyr);
    foot_gyr_buf.push_back(foot_gyr);
    jang_buf.push_back(jang);
    jvel_buf.push_back(jvel);

    propagate(dt, body_gyr, foot_gyr, jang, jvel);
  }

  void repropagate(const Eigen::Vector3d& _linearized_bg, const Eigen::Vector3d& _linearized_bf, const Eigen::Vector3d& _linearized_bv,
                   const Vec_rho& _linearized_rho) {
    sum_dt = 0.0;
    body_gyr_0 = body_gyr_init;
    foot_gyr_0 = foot_gyr_init;
    jang_0 = jang_init;
    jvel_0 = jvel_init;

    delta_epsilon.setZero();
    delta_q.setIdentity();
    linearized_bg = _linearized_bg;
    linearized_bf = _linearized_bf;
    linearized_bv = _linearized_bv;
    linearized_rho = _linearized_rho;
    jacobian.setIdentity();
    covariance.setZero();

    for (int i = 0; i < static_cast<int>(dt_buf.size()); i++) {
      propagate(dt_buf[i], body_gyr_buf[i], foot_gyr_buf[i], jang_buf[i], jvel_buf[i]);
    }
  }

  void midPointIntegration(double _dt,                                                               // time
                           const Eigen::Vector3d& _body_gyr_0, const Eigen::Vector3d& _body_gyr_1,   // body IMU
                           const Eigen::Vector3d& _foot_gyr_0, const Eigen::Vector3d& _foot_gyr_1,   // foot IMU
                           const Eigen::Vector3d& _jang_0, const Eigen::Vector3d& _jang_1,           // leg angles
                           const Eigen::Vector3d& _jvel_0, const Eigen::Vector3d& _jvel_1,           // leg angle velocity
                           const Eigen::Vector3d& delta_epsilon, const Eigen::Quaterniond& delta_q,  // previous integration terms
                           const Eigen::Vector3d& linearized_bg, const Eigen::Vector3d& linearized_bf, const Eigen::Vector3d& linearized_bv,
                           const Vec_rho& linearized_rho,                                              // previous biases
                           Eigen::Vector3d& result_delta_epsilon, Eigen::Quaterniond& result_delta_q,  // result integration terms
                           Eigen::Vector3d& result_linearized_bg, Eigen::Vector3d& result_linearized_bf,
                           Eigen::Vector3d& result_linearized_bv, Vec_rho& result_linearized_rho,  // result biases
                           bool update_jacobian) {
    // orientation integration
    Eigen::Vector3d un_gyr = 0.5 * (_body_gyr_0 + _body_gyr_1) - linearized_bg;
    result_delta_q = delta_q * Eigen::Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
    result_linearized_bg = linearized_bg;

    // LO velocity integration
    Eigen::Vector3d vi;
    vi.setZero();
    vi = tightUtils->calBodyVel(leg_id, _jang_0, _jvel_0, _body_gyr_0, _foot_gyr_0, linearized_bg, linearized_bf, linearized_bv,
                                linearized_rho(0));
    Eigen::Vector3d vip1;
    vip1.setZero();
    vip1 = tightUtils->calBodyVel(leg_id, _jang_1, _jvel_1, _body_gyr_1, _foot_gyr_1, linearized_bg, linearized_bf, linearized_bv,
                                  linearized_rho(0));
    result_linearized_bf = linearized_bf;
    result_linearized_bv = linearized_bv;
    result_linearized_rho = linearized_rho;

    Eigen::Matrix3d delta_R = delta_q.toRotationMatrix();
    Eigen::Matrix3d result_delta_R = result_delta_q.toRotationMatrix();

    result_delta_epsilon = delta_epsilon + 0.5 * (delta_R * vi + result_delta_R * vip1) * _dt;

    if (update_jacobian) {
      Eigen::Matrix3d R_w_x = legged::skewSymmetric(un_gyr);
      Eigen::Matrix3d f11 = Eigen::Matrix3d::Identity() - R_w_x * _dt;
      Eigen::Matrix3d f21 = -0.5 * _dt * (delta_R * legged::skewSymmetric(vi) + result_delta_R * legged::skewSymmetric(vip1)) * f11;

      // epsilon derivative respect to bg
      Eigen::Matrix3d dvdbgi;
      dvdbgi.setZero();
      dvdbgi = tightUtils->calBodyVelDbg(leg_id, _jang_0, _jvel_0, _body_gyr_0, _foot_gyr_0, linearized_bg, linearized_bf, linearized_bv,
                                         linearized_rho(0));
      Eigen::Matrix3d dvdbgip1;
      dvdbgip1.setZero();
      dvdbgip1 = tightUtils->calBodyVelDbg(leg_id, _jang_1, _jvel_1, _body_gyr_1, _foot_gyr_1, linearized_bg, linearized_bf, linearized_bv,
                                           linearized_rho(0));
      Eigen::Matrix3d f23 = 0.5 * _dt * (delta_R * dvdbgi + result_delta_R * dvdbgip1);

      // epsilon derivative respect to bf
      Eigen::Matrix3d dvdbfi;
      dvdbfi.setZero();
      dvdbfi = tightUtils->calBodyVelDbf(leg_id, _jang_0, _jvel_0, _body_gyr_0, _foot_gyr_0, linearized_bg, linearized_bf, linearized_bv,
                                         linearized_rho(0));
      Eigen::Matrix3d dvdbfip1;
      dvdbfip1.setZero();
      dvdbfip1 = tightUtils->calBodyVelDbf(leg_id, _jang_1, _jvel_1, _body_gyr_1, _foot_gyr_1, linearized_bg, linearized_bf, linearized_bv,
                                           linearized_rho(0));
      Eigen::Matrix3d f24 = 0.5 * _dt * (delta_R * dvdbfi + result_delta_R * dvdbfip1);

      // epsilon derivative respect to bv
      Eigen::Matrix3d dvdbvi;
      dvdbvi.setZero();
      dvdbvi = tightUtils->calBodyVelDbv(leg_id, _jang_0, _jvel_0, _body_gyr_0, _foot_gyr_0, linearized_bg, linearized_bf, linearized_bv,
                                         linearized_rho(0));
      Eigen::Matrix3d dvdbvip1;
      dvdbvip1.setZero();
      dvdbvip1 = tightUtils->calBodyVelDbv(leg_id, _jang_1, _jvel_1, _body_gyr_1, _foot_gyr_1, linearized_bg, linearized_bf, linearized_bv,
                                           linearized_rho(0));
      Eigen::Matrix3d f25 = 0.5 * _dt * (delta_R * dvdbvi + result_delta_R * dvdbvip1);

      // epsilon derivative respect to rho
      Eigen::Matrix<double, 3, 1> dvdbrhoi;
      dvdbrhoi.setZero();
      dvdbrhoi = tightUtils->calBodyVelDd0(leg_id, _jang_0, _jvel_0, _body_gyr_0, _foot_gyr_0, linearized_bg, linearized_bf, linearized_bv,
                                           linearized_rho(0));
      Eigen::Matrix<double, 3, 1> dvdbrhoip1;
      dvdbrhoip1.setZero();
      dvdbrhoip1 = tightUtils->calBodyVelDd0(leg_id, _jang_1, _jvel_1, _body_gyr_1, _foot_gyr_1, linearized_bg, linearized_bf,
                                             linearized_bv, linearized_rho(0));
      Eigen::Matrix<double, 3, 1> f26 = 0.5 * _dt * (delta_R * dvdbrhoi + result_delta_R * dvdbrhoip1);

      // construct jacobian
      F.setZero();
      F.block<3, 3>(T_R, T_R) = f11;
      F.block<3, 3>(T_R, T_BG) = -Eigen::Matrix3d::Identity() * _dt;

      F.block<3, 3>(T_E, T_R) = f21;
      F.block<3, 3>(T_E, T_E) = Eigen::Matrix3d::Identity() * _dt;
      F.block<3, 3>(T_E, T_BG) = f23;
      F.block<3, 3>(T_E, T_BF) = f24;
      F.block<3, 3>(T_E, T_BV) = f25;
      F.block<3, RHO_SIZE>(T_E, T_RHO) = f26;

      F.block<3, 3>(T_BG, T_BG) = Eigen::Matrix3d::Identity() * _dt;
      F.block<3, 3>(T_BF, T_BF) = Eigen::Matrix3d::Identity() * _dt;
      F.block<3, 3>(T_BV, T_BV) = Eigen::Matrix3d::Identity() * _dt;
      F.block<RHO_SIZE, RHO_SIZE>(T_RHO, T_RHO) = Eigen::Matrix<double, RHO_SIZE, RHO_SIZE>::Identity() * _dt;

      // noise related jacobian
      V.setZero();
      // epsilon derivative respect to jang
      Eigen::Matrix3d dv_djang_i;
      dv_djang_i.setZero();
      dv_djang_i = tightUtils->calBodyVelDjang(leg_id, _jang_0, _jvel_0, _body_gyr_0, _foot_gyr_0, linearized_bg, linearized_bf,
                                               linearized_bv, linearized_rho(0));
      Eigen::Matrix3d dv_djang_ip1;
      dv_djang_ip1.setZero();
      dv_djang_ip1 = tightUtils->calBodyVelDjang(leg_id, _jang_1, _jvel_1, _body_gyr_1, _foot_gyr_1, linearized_bg, linearized_bf,
                                                 linearized_bv, linearized_rho(0));
      Eigen::Matrix3d V21 = 0.5 * _dt * (delta_R * dv_djang_i + result_delta_R * dv_djang_ip1);
      // epsilon derivative respect to jvel
      Eigen::Matrix3d dv_djvel_i;
      dv_djvel_i.setZero();
      dv_djvel_i = tightUtils->calBodyVelDjvel(leg_id, _jang_0, _jvel_0, _body_gyr_0, _foot_gyr_0, linearized_bg, linearized_bf,
                                               linearized_bv, linearized_rho(0));
      Eigen::Matrix3d dv_djvel_ip1;
      dv_djvel_ip1.setZero();
      dv_djvel_ip1 = tightUtils->calBodyVelDjvel(leg_id, _jang_1, _jvel_1, _body_gyr_1, _foot_gyr_1, linearized_bg, linearized_bf,
                                                 linearized_bv, linearized_rho(0));
      Eigen::Matrix3d V22 = 0.5 * _dt * (delta_R * dv_djvel_i + result_delta_R * dv_djvel_ip1);
      // epsilon derivative respect to bodyGyr
      Eigen::Matrix3d dv_dbodyGyr_i;
      dv_dbodyGyr_i.setZero();
      dv_dbodyGyr_i = tightUtils->calBodyVelDbodyGyr(leg_id, _jang_0, _jvel_0, _body_gyr_0, _foot_gyr_0, linearized_bg, linearized_bf,
                                                     linearized_bv, linearized_rho(0));
      Eigen::Matrix3d dv_dbodyGyr_ip1;
      dv_dbodyGyr_ip1.setZero();
      dv_dbodyGyr_ip1 = tightUtils->calBodyVelDbodyGyr(leg_id, _jang_1, _jvel_1, _body_gyr_1, _foot_gyr_1, linearized_bg, linearized_bf,
                                                       linearized_bv, linearized_rho(0));
      Eigen::Matrix3d V23 = 0.5 * _dt * (delta_R * dv_dbodyGyr_i + result_delta_R * dv_dbodyGyr_ip1);
      // epsilon derivative respect to footGyr
      Eigen::Matrix3d dv_dfootGyr_i;
      dv_dfootGyr_i.setZero();
      dv_dfootGyr_i = tightUtils->calBodyVelDfootGyr(leg_id, _jang_0, _jvel_0, _body_gyr_0, _foot_gyr_0, linearized_bg, linearized_bf,
                                                     linearized_bv, linearized_rho(0));
      Eigen::Matrix3d dv_dfootGyr_ip1;
      dv_dfootGyr_ip1.setZero();
      dv_dfootGyr_ip1 = tightUtils->calBodyVelDfootGyr(leg_id, _jang_1, _jvel_1, _body_gyr_1, _foot_gyr_1, linearized_bg, linearized_bf,
                                                       linearized_bv, linearized_rho(0));
      Eigen::Matrix3d V25 = 0.5 * _dt * (delta_R * dv_dfootGyr_i + result_delta_R * dv_dfootGyr_ip1);

      V.setZero();
      // signs are not matter actually
      V.block<3, 3>(T_R, T_GN) = -1.0 * Eigen::Matrix3d::Identity() * _dt;
      V.block<3, 3>(T_E, T_PHIN) = V21;
      V.block<3, 3>(T_E, T_DPHIN) = V22;
      V.block<3, 3>(T_E, T_GN) = V23;
      V.block<3, 3>(T_E, T_FN) = V25;

      V.block<3, 3>(T_BG, T_GW) = -1.0 * Eigen::Matrix3d::Identity() * _dt;
      V.block<3, 3>(T_BF, T_FW) = -1.0 * Eigen::Matrix3d::Identity() * _dt;
      V.block<3, 3>(T_BV, T_VN) = -1.0 * Eigen::Matrix3d::Identity() * _dt;
      V.block<RHO_SIZE, RHO_SIZE>(T_RHO, TRHO_N) = -1.0 * Eigen::Matrix<double, RHO_SIZE, RHO_SIZE>::Identity() * _dt;

      jacobian = F * jacobian;
      covariance = F * covariance * F.transpose() + V * noise_diag * V.transpose();
    }
  }

  void propagate(const double _dt, const Eigen::Vector3d& _body_gyr_1, const Eigen::Vector3d& _foot_gyr_1, const Eigen::Vector3d& _jang_1,
                 const Eigen::Vector3d& _jvel_1) {
    dt = _dt;
    body_gyr_1 = _body_gyr_1;
    foot_gyr_1 = _foot_gyr_1;
    jang_1 = _jang_1;
    jvel_1 = _jvel_1;

    Eigen::Vector3d result_delta_epsilon;
    Eigen::Quaterniond result_delta_q;

    Eigen::Vector3d result_linearized_bg;
    Eigen::Vector3d result_linearized_bf;
    Eigen::Vector3d result_linearized_bv;
    Vec_rho result_linearized_rho;

    midPointIntegration(dt, body_gyr_0, body_gyr_1, foot_gyr_0, foot_gyr_1, jang_0, jang_1, jvel_0, jvel_1, delta_epsilon, delta_q,
                        linearized_bg, linearized_bf, linearized_bv, linearized_rho, result_delta_epsilon, result_delta_q,
                        result_linearized_bg, result_linearized_bf, result_linearized_bv, result_linearized_rho, 1);

    delta_epsilon = result_delta_epsilon;
    delta_q = result_delta_q;
    delta_q.normalize();
    linearized_bg = result_linearized_bg;
    linearized_bf = result_linearized_bf;
    linearized_bv = result_linearized_bv;
    linearized_rho = result_linearized_rho;

    sum_dt += dt;
    body_gyr_0 = body_gyr_1;
    foot_gyr_0 = foot_gyr_1;
    jang_0 = jang_1;
    jvel_0 = jvel_1;
  }

  Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, 1> evaluate(const Eigen::Vector3d& Pi, const Eigen::Quaterniond& Qi,
                                                            const Eigen::Vector3d& Bgi, const Eigen::Vector3d& Bfi,
                                                            const Eigen::Vector3d& Bvi, const Vec_rho& rhoi, const Eigen::Vector3d& Pj,
                                                            const Eigen::Quaterniond& Qj, const Eigen::Vector3d& Bgj,
                                                            const Eigen::Vector3d& Bfj, const Eigen::Vector3d& Bvj, const Vec_rho& rhoj) {
    Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, 1> residuals;
    residuals.setZero();

    Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(T_R, T_BG);

    Eigen::Vector3d dbg = Bgi - linearized_bg;

    Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);

    residuals.block<3, 1>(T_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();

    Eigen::Matrix3d de_dbg = jacobian.block<3, 3>(T_E, T_BG);
    Eigen::Matrix3d de_dbf = jacobian.block<3, 3>(T_E, T_BF);
    Eigen::Matrix3d de_dbv = jacobian.block<3, 3>(T_E, T_BV);
    Eigen::Vector3d de_drho = jacobian.block<3, 1>(T_E, T_RHO);

    Eigen::Vector3d dbf = Bfi - linearized_bf;
    Eigen::Vector3d dbv = Bvi - linearized_bv;
    Vec_rho drho = rhoi - linearized_rho;

    Eigen::Vector3d corrected_delta_epsilon = delta_epsilon + de_dbg * dbg + de_dbf * dbf + de_dbv * dbv + de_drho * drho;

    residuals.block<3, 1>(T_E, 0) = Qi.inverse() * (Pj - Pi) - corrected_delta_epsilon;

    residuals.block<3, 1>(T_BG, 0) = Bgj - Bgi;
    residuals.block<3, 1>(T_BF, 0) = Bfj - Bfi;
    residuals.block<3, 1>(T_BV, 0) = Bvj - Bvi;
    residuals.block<RHO_SIZE, 1>(T_RHO, 0) = rhoj - rhoi;

    return residuals;
  }

  double sum_dt;
  Eigen::Vector3d delta_epsilon;
  Eigen::Quaterniond delta_q;
  Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE> jacobian, covariance;

  Eigen::Vector3d linearized_bg, linearized_bf, linearized_bv;
  Vec_rho linearized_rho;

 private:
  int leg_id;  // 0 FL, 1 FR, 2 RL, 3 RR, very important when calling LOTightUtils
  double dt;
  Eigen::Vector3d body_gyr_0, body_gyr_1;
  Eigen::Vector3d foot_gyr_0, foot_gyr_1;
  Eigen::Vector3d jang_0, jang_1;
  Eigen::Vector3d jvel_0, jvel_1;
  Eigen::Vector3d body_gyr_init, foot_gyr_init, jang_init, jvel_init;

  std::vector<double> dt_buf;
  std::vector<Eigen::Vector3d> body_gyr_buf;
  std::vector<Eigen::Vector3d> foot_gyr_buf;
  std::vector<Eigen::Vector3d> jang_buf;
  std::vector<Eigen::Vector3d> jvel_buf;

  Eigen::DiagonalMatrix<double, LO_TIGHT_NOISE_SIZE> noise_diag;

  // helper functions for calculating jacobian
  Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE> F;
  Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_NOISE_SIZE> V;

  LOTightUtils* tightUtils;
};