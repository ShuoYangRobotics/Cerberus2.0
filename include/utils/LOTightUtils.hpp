#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <casadi/casadi.hpp>

#include "mipo/MIPOEstimator.hpp"
#include "utils/casadi_kino.hpp"
#include "utils/utils.hpp"

/*
 * This class use casadi to calculate body velocity observation from leg joint angle and velocity, and foot gyro. It is used in
 * LOTightIntegrationBase to do contact preintegration.
 */

class LOTightUtils {
 public:
  LOTightUtils() { init(); }
  ~LOTightUtils();

  Eigen::Vector3d calBodyVel(const int leg_id, const Eigen::Vector3d& jang, const Eigen::Vector3d& jvel, const Eigen::Vector3d& body_gyr,
                             const Eigen::Vector3d& foot_gyr, const Eigen::Vector3d& linearized_bg, const Eigen::Vector3d& linearized_bf,
                             const Eigen::Vector3d& linearized_bv, const double d0) {
    // convert input to casadi::DM and call casadi function v_fun
    casadi::DM jang_dm = Utils::eig_to_cas_DM<3, 1>(jang);
    casadi::DM jvel_dm = Utils::eig_to_cas_DM<3, 1>(jvel);
    casadi::DM body_gyr_dm = Utils::eig_to_cas_DM<3, 1>(body_gyr);
    casadi::DM foot_gyr_dm = Utils::eig_to_cas_DM<3, 1>(foot_gyr);
    casadi::DM linearized_bg_dm = Utils::eig_to_cas_DM<3, 1>(linearized_bg);
    casadi::DM linearized_bf_dm = Utils::eig_to_cas_DM<3, 1>(linearized_bf);
    casadi::DM linearized_bv_dm = Utils::eig_to_cas_DM<3, 1>(linearized_bv);
    casadi::DM d0_dm = d0;

    casadi::DM kin_dm = Utils::eig_to_cas_DM<5, NUM_LEG>(kin_true_);
    casadi::DM R_fi_dm = Utils::eig_to_cas_DM<3, 3>(mipo_utils.R_fi_list[leg_id]);

    std::vector<casadi::DM> arg = {jang_dm, jvel_dm, body_gyr_dm, linearized_bg_dm, foot_gyr_dm, linearized_bf_dm,
                                   kin_dm,  R_fi_dm, d0_dm,       linearized_bv_dm};
    std::vector<casadi::DM> res = v_fun[leg_id](arg);

    Eigen::Matrix<double, 3, 1> body_v = Utils::cas_DM_to_eig_mat<3, 1>(res[0]);

    return body_v;
  }

 private:
  void init() {
    kin_true_ << 0.1805, 0.1805, -0.1805, -0.1805, 0.047, -0.047, 0.047, -0.047, 0.0838, -0.0838, 0.0838, -0.0838, 0.2, 0.2, 0.2, 0.2, 0.2,
        0.2, 0.2, 0.2;
    // define casadi variables
    casadi::SX j_ang = casadi::SX::sym("j_ang", 3);       // joint angle
    casadi::SX j_vel = casadi::SX::sym("j_vel", 3);       // joint velocity
    casadi::SX body_w = casadi::SX::sym("body_w", 3);     // body angular velocity
    casadi::SX bg = casadi::SX::sym("bg", 3);             // body angular velocity bias
    casadi::SX foot_w = casadi::SX::sym("foot_w", 3);     // foot angular velocity
    casadi::SX bf = casadi::SX::sym("bf", 3);             // foot angular velocity bias
    casadi::SX kin = casadi::SX::sym("kin", 5, NUM_LEG);  // leg kinematics
    casadi::SX R_fi = casadi::SX::sym("R_fi", 3, 3);      // foot IMU to foot frame rotation
    casadi::SX d0 = casadi::SX::sym("d0", 1);             // pivoting model parameter
    casadi::SX bv = casadi::SX::sym("bv", 3);             // inferred velocity bias
    // convert casadi variables to eigen vectors and matrices
    auto j_ang_eig_X = Utils::cas_to_eig(j_ang);
    Eigen::Matrix<casadi::SX, 3, 1> j_ang_eig = j_ang_eig_X.head(3);
    auto j_vel_eig_X = Utils::cas_to_eig(j_vel);
    Eigen::Matrix<casadi::SX, 3, 1> j_vel_eig = j_vel_eig_X.head(3);
    auto body_w_eig_X = Utils::cas_to_eig(body_w);
    Eigen::Matrix<casadi::SX, 3, 1> body_w_eig = body_w_eig_X.head(3);
    auto bg_eig_X = Utils::cas_to_eig(bg);
    Eigen::Matrix<casadi::SX, 3, 1> bg_eig = bg_eig_X.head(3);
    auto foot_w_eig_X = Utils::cas_to_eig(foot_w);
    Eigen::Matrix<casadi::SX, 3, 1> foot_w_eig = foot_w_eig_X.head(3);
    auto bf_eig_X = Utils::cas_to_eig(bf);
    Eigen::Matrix<casadi::SX, 3, 1> bf_eig = bf_eig_X.head(3);
    auto bv_eig_X = Utils::cas_to_eig(bv);
    Eigen::Matrix<casadi::SX, 3, 1> bv_eig = bv_eig_X.head(3);

    Eigen::Matrix<casadi::SX, 5, NUM_LEG> kin_eig = Utils::cas_to_eigmat<5, NUM_LEG>(kin);
    Eigen::Matrix<casadi::SX, 3, 3> R_fi_eig = Utils::cas_to_eigmat<3, 3>(R_fi);

    for (int i = 0; i < NUM_LEG; i++) {
      body_vel_eig_exp[i] = getBodyVel(i, j_ang_eig, j_vel_eig, body_w_eig, bg_eig, foot_w_eig, bf_eig, kin_eig, R_fi_eig, d0, bv_eig);
      body_vel_exp[i] = Utils::eig_to_cas(body_vel_eig_exp[i]);

      dvdphi[i] = jacobian(body_vel_exp[i], j_ang);
      dvddphi[i] = jacobian(body_vel_exp[i], j_vel);
      dvdw[i] = jacobian(body_vel_exp[i], body_w);
      dvdwf[i] = jacobian(body_vel_exp[i], foot_w);
      dvdbg[i] = jacobian(body_vel_exp[i], bg);
      dvdbf[i] = jacobian(body_vel_exp[i], bf);
      dvdbv[i] = jacobian(body_vel_exp[i], bv);
      dvdd0[i] = jacobian(body_vel_exp[i], d0);

      // define functions
      v_fun[i] = casadi::Function("v_fun", {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv}, {body_vel_exp[i]});
      dvdphi_fun[i] = casadi::Function("dvdphi_fun", {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv}, {dvdphi[i]});
      dvddphi_fun[i] = casadi::Function("dvddphi_fun", {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv}, {dvddphi[i]});
      dvdw_fun[i] = casadi::Function("dvdw_fun", {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv}, {dvdw[i]});
      dvdwf_fun[i] = casadi::Function("dvdwf_fun", {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv}, {dvdwf[i]});
      dvdbg_fun[i] = casadi::Function("dvdbg_fun", {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv}, {dvdbg[i]});
      dvdbf_fun[i] = casadi::Function("dvdbf_fun", {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv}, {dvdbf[i]});
      dvdbv_fun[i] = casadi::Function("dvdbv_fun", {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv}, {dvdbv[i]});
      dvdd0_fun[i] = casadi::Function("dvdd0_fun", {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv}, {dvdd0[i]});
    }
  }

  // get body velocity from leg joint angle and velocity, and foot gyro, the body velocity is expressed in body frame
  Eigen::Matrix<casadi::SX, 3, 1> getBodyVel(int leg_id, const Eigen::Matrix<casadi::SX, 3, 1>& j_ang,
                                             const Eigen::Matrix<casadi::SX, 3, 1>& j_vel, const Eigen::Matrix<casadi::SX, 3, 1>& body_w,
                                             const Eigen::Matrix<casadi::SX, 3, 1>& bg, const Eigen::Matrix<casadi::SX, 3, 1>& foot_w,
                                             const Eigen::Matrix<casadi::SX, 3, 1>& bf,
                                             const Eigen::Matrix<casadi::SX, Eigen::Dynamic, NUM_LEG> kin_param,
                                             const Eigen::Matrix<casadi::SX, 3, 3>& R_fi, const casadi::SX d0,
                                             const Eigen::Matrix<casadi::SX, 3, 1>& bv) {
    Eigen::Matrix<casadi::SX, -1, 1> kin_param_leg = kin_param.col(leg_id);
    Eigen::Matrix<casadi::SX, 3, 1> p_rf = legged::fk_pf_pos(j_ang, kin_param_leg);
    Eigen::Matrix<casadi::SX, 3, 3> J_rf = legged::d_fk_dt(j_ang, kin_param_leg);
    Eigen::Matrix<casadi::SX, 3, 3> R_bf = legged::fk_pf_rot(j_ang);

    Eigen::Matrix<casadi::SX, 3, 1> body_w_no_bias = body_w - bg;
    Eigen::Matrix<casadi::SX, 3, 1> foot_w_no_bias = foot_w - bf;
    // velocity from joints
    Eigen::Matrix<casadi::SX, 3, 1> leg_v = -(J_rf * j_vel + legged::skewSymmetric<casadi::SX>(body_w_no_bias) * p_rf);
    // velocity from foot pivoting model
    Eigen::Matrix<casadi::SX, 3, 1> foot_v_pivoting =
        R_bf * R_fi * legged::skewSymmetric<casadi::SX>(foot_w_no_bias) * (-d0 * p_rf / p_rf.norm());

    // be careful about the sign of terms
    return leg_v + foot_v_pivoting - bv;
  }

  MIPOEstimatorSensorData mipo_utils;           // we only use R_fi_list in it
  Eigen::Matrix<double, 5, NUM_LEG> kin_true_;  // use kinematics

  Eigen::Matrix<casadi::SX, 3, 1> body_vel_eig_exp[NUM_LEG];
  casadi::SX body_vel_exp[NUM_LEG];
  // a number of derivative quantities
  casadi::SX dvdphi[NUM_LEG];
  casadi::SX dvddphi[NUM_LEG];
  casadi::SX dvdw[NUM_LEG];
  casadi::SX dvdwf[NUM_LEG];

  casadi::SX dvdbg[NUM_LEG];
  casadi::SX dvdbf[NUM_LEG];
  casadi::SX dvdbv[NUM_LEG];
  casadi::SX dvdd0[NUM_LEG];

  // casadi functions
  casadi::Function v_fun[NUM_LEG];
  casadi::Function dvdphi_fun[NUM_LEG];
  casadi::Function dvddphi_fun[NUM_LEG];
  casadi::Function dvdw_fun[NUM_LEG];
  casadi::Function dvdwf_fun[NUM_LEG];
  casadi::Function dvdbg_fun[NUM_LEG];
  casadi::Function dvdbf_fun[NUM_LEG];
  casadi::Function dvdbv_fun[NUM_LEG];
  casadi::Function dvdd0_fun[NUM_LEG];
};