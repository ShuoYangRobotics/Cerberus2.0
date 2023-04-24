#pragma once
// for checking file existence
#include <sys/stat.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <mutex>

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
  ~LOTightUtils(){};

  // notice that the body velocity is in the body frame
  Eigen::Vector3d calBodyVel(const int leg_id, const Eigen::Vector3d& jang, const Eigen::Vector3d& jvel, const Eigen::Vector3d& body_gyr,
                             const Eigen::Vector3d& foot_gyr, const Eigen::Vector3d& linearized_bg, const Eigen::Vector3d& linearized_bf,
                             const Eigen::Vector3d& linearized_bv, const double d0) {
    mtx.lock();
    eig_args_asm(leg_id, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);

    casadi::Function tmp = gen_v_fun[leg_id];
    std::vector<casadi::DM> res = tmp(arg);

    Eigen::Matrix<double, 3, 1> body_v = Utils::cas_DM_to_eig_mat<3, 1>(res[0]);

    mtx.unlock();
    return body_v;
  }

  // similar functions to get derivatives, could define some macros for this
  Eigen::Matrix3d calBodyVelDjang(const int leg_id, const Eigen::Vector3d& jang, const Eigen::Vector3d& jvel,
                                  const Eigen::Vector3d& body_gyr, const Eigen::Vector3d& foot_gyr, const Eigen::Vector3d& linearized_bg,
                                  const Eigen::Vector3d& linearized_bf, const Eigen::Vector3d& linearized_bv, const double d0) {
    mtx.lock();
    eig_args_asm(leg_id, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);

    casadi::Function tmp = gen_dvdphi_fun[leg_id];
    std::vector<casadi::DM> res = tmp(arg);

    Eigen::Matrix<double, 3, 3> dvdphi_jac = Utils::cas_DM_to_eig_mat<3, 3>(res[0]);

    mtx.unlock();
    return dvdphi_jac;
  }

  Eigen::Matrix3d calBodyVelDjvel(const int leg_id, const Eigen::Vector3d& jang, const Eigen::Vector3d& jvel,
                                  const Eigen::Vector3d& body_gyr, const Eigen::Vector3d& foot_gyr, const Eigen::Vector3d& linearized_bg,
                                  const Eigen::Vector3d& linearized_bf, const Eigen::Vector3d& linearized_bv, const double d0) {
    mtx.lock();
    eig_args_asm(leg_id, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);

    casadi::Function tmp = gen_dvddphi_fun[leg_id];
    std::vector<casadi::DM> res = tmp(arg);

    Eigen::Matrix<double, 3, 3> dvddphi_jac = Utils::cas_DM_to_eig_mat<3, 3>(res[0]);

    mtx.unlock();
    return dvddphi_jac;
  }

  Eigen::Matrix3d calBodyVelDbodyGyr(const int leg_id, const Eigen::Vector3d& jang, const Eigen::Vector3d& jvel,
                                     const Eigen::Vector3d& body_gyr, const Eigen::Vector3d& foot_gyr, const Eigen::Vector3d& linearized_bg,
                                     const Eigen::Vector3d& linearized_bf, const Eigen::Vector3d& linearized_bv, const double d0) {
    mtx.lock();
    eig_args_asm(leg_id, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);

    casadi::Function tmp = gen_dvdw_fun[leg_id];
    std::vector<casadi::DM> res = tmp(arg);

    Eigen::Matrix<double, 3, 3> dvdw_jac = Utils::cas_DM_to_eig_mat<3, 3>(res[0]);

    mtx.unlock();
    return dvdw_jac;
  }

  Eigen::Matrix3d calBodyVelDfootGyr(const int leg_id, const Eigen::Vector3d& jang, const Eigen::Vector3d& jvel,
                                     const Eigen::Vector3d& body_gyr, const Eigen::Vector3d& foot_gyr, const Eigen::Vector3d& linearized_bg,
                                     const Eigen::Vector3d& linearized_bf, const Eigen::Vector3d& linearized_bv, const double d0) {
    mtx.lock();
    eig_args_asm(leg_id, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);

    casadi::Function tmp = gen_dvdwf_fun[leg_id];
    std::vector<casadi::DM> res = tmp(arg);

    Eigen::Matrix<double, 3, 3> dvdwf_jac = Utils::cas_DM_to_eig_mat<3, 3>(res[0]);

    mtx.unlock();
    return dvdwf_jac;
  }

  Eigen::Matrix3d calBodyVelDbg(const int leg_id, const Eigen::Vector3d& jang, const Eigen::Vector3d& jvel, const Eigen::Vector3d& body_gyr,
                                const Eigen::Vector3d& foot_gyr, const Eigen::Vector3d& linearized_bg, const Eigen::Vector3d& linearized_bf,
                                const Eigen::Vector3d& linearized_bv, const double d0) {
    mtx.lock();
    eig_args_asm(leg_id, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);

    casadi::Function tmp = gen_dvdbg_fun[leg_id];
    std::vector<casadi::DM> res = tmp(arg);

    Eigen::Matrix<double, 3, 3> dvdbg_jac = Utils::cas_DM_to_eig_mat<3, 3>(res[0]);

    mtx.unlock();
    return dvdbg_jac;
  }

  Eigen::Matrix3d calBodyVelDbf(const int leg_id, const Eigen::Vector3d& jang, const Eigen::Vector3d& jvel, const Eigen::Vector3d& body_gyr,
                                const Eigen::Vector3d& foot_gyr, const Eigen::Vector3d& linearized_bg, const Eigen::Vector3d& linearized_bf,
                                const Eigen::Vector3d& linearized_bv, const double d0) {
    mtx.lock();
    eig_args_asm(leg_id, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);

    casadi::Function tmp = gen_dvdbf_fun[leg_id];
    std::vector<casadi::DM> res = tmp(arg);

    Eigen::Matrix<double, 3, 3> dvdbf_jac = Utils::cas_DM_to_eig_mat<3, 3>(res[0]);

    mtx.unlock();
    return dvdbf_jac;
  }

  Eigen::Matrix3d calBodyVelDbv(const int leg_id, const Eigen::Vector3d& jang, const Eigen::Vector3d& jvel, const Eigen::Vector3d& body_gyr,
                                const Eigen::Vector3d& foot_gyr, const Eigen::Vector3d& linearized_bg, const Eigen::Vector3d& linearized_bf,
                                const Eigen::Vector3d& linearized_bv, const double d0) {
    mtx.lock();
    eig_args_asm(leg_id, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);

    casadi::Function tmp = gen_dvdbv_fun[leg_id];
    std::vector<casadi::DM> res = tmp(arg);

    Eigen::Matrix<double, 3, 3> dvdbv_jac = Utils::cas_DM_to_eig_mat<3, 3>(res[0]);

    mtx.unlock();
    return dvdbv_jac;
  }

  Eigen::Matrix<double, 3, 1> calBodyVelDd0(const int leg_id, const Eigen::Vector3d& jang, const Eigen::Vector3d& jvel,
                                            const Eigen::Vector3d& body_gyr, const Eigen::Vector3d& foot_gyr,
                                            const Eigen::Vector3d& linearized_bg, const Eigen::Vector3d& linearized_bf,
                                            const Eigen::Vector3d& linearized_bv, const double d0) {
    mtx.lock();
    eig_args_asm(leg_id, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);

    casadi::Function tmp = gen_dvdd0_fun[leg_id];
    std::vector<casadi::DM> res = tmp(arg);

    Eigen::Matrix<double, 3, 1> dvdd0_jac = Utils::cas_DM_to_eig_mat<3, 1>(res[0]);

    mtx.unlock();
    return dvdd0_jac;
  }

 private:
  void init() {
    kin_true_ << 0.1805, 0.1805, -0.1805, -0.1805, 0.047, -0.047, 0.047, -0.047, 0.0838, -0.0838, 0.0838, -0.0838, 0.2, 0.2, 0.2, 0.2, 0.2,
        0.2, 0.2, 0.2;

    // generate functions as shared lib
    // this is only done once. in the CMakelists.txt, the last test executable is set to generate the shared lib
    if (if_code_gen() == false) {
      v_fun.clear();
      dvdphi_fun.clear();
      dvddphi_fun.clear();
      dvdw_fun.clear();
      dvdwf_fun.clear();
      dvdbg_fun.clear();
      dvdbf_fun.clear();
      dvdbv_fun.clear();
      dvdd0_fun.clear();
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
      auto d0_eig_X = Utils::cas_to_eig(d0);
      Eigen::Matrix<casadi::SX, 1, 1> d0_eig = d0_eig_X.head(1);

      Eigen::Matrix<casadi::SX, 5, NUM_LEG> kin_eig = Utils::cas_to_eigmat<5, NUM_LEG>(kin);
      Eigen::Matrix<casadi::SX, 3, 3> R_fi_eig = Utils::cas_to_eigmat<3, 3>(R_fi);

      for (int i = 0; i < NUM_LEG; i++) {
        body_vel_eig_exp[i] =
            getBodyVel(i, j_ang_eig, j_vel_eig, body_w_eig, bg_eig, foot_w_eig, bf_eig, kin_eig, R_fi_eig, d0_eig, bv_eig);
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
        v_fun.push_back(casadi::Function(fun_name_list[0] + std::to_string(i), {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv},
                                         {body_vel_exp[i]}));
        dvdphi_fun.push_back(
            casadi::Function(fun_name_list[1] + std::to_string(i), {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv}, {dvdphi[i]}));
        dvddphi_fun.push_back(casadi::Function(fun_name_list[2] + std::to_string(i),
                                               {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv}, {dvddphi[i]}));
        dvdw_fun.push_back(
            casadi::Function(fun_name_list[3] + std::to_string(i), {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv}, {dvdw[i]}));
        dvdwf_fun.push_back(
            casadi::Function(fun_name_list[4] + std::to_string(i), {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv}, {dvdwf[i]}));
        dvdbg_fun.push_back(
            casadi::Function(fun_name_list[5] + std::to_string(i), {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv}, {dvdbg[i]}));
        dvdbf_fun.push_back(
            casadi::Function(fun_name_list[6] + std::to_string(i), {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv}, {dvdbf[i]}));
        dvdbv_fun.push_back(
            casadi::Function(fun_name_list[7] + std::to_string(i), {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv}, {dvdbv[i]}));
        dvdd0_fun.push_back(
            casadi::Function(fun_name_list[8] + std::to_string(i), {j_ang, j_vel, body_w, bg, foot_w, bf, kin, R_fi, d0, bv}, {dvdd0[i]}));
      }
      for (int i = 0; i < NUM_LEG; i++) {
        codegen(v_fun[i]);
        codegen(dvdphi_fun[i]);
        codegen(dvddphi_fun[i]);
        codegen(dvdw_fun[i]);
        codegen(dvdwf_fun[i]);
        codegen(dvdbg_fun[i]);
        codegen(dvdbf_fun[i]);
        codegen(dvdbv_fun[i]);
        codegen(dvdd0_fun[i]);
      }
    }

    // load codegen funcs
    gen_v_fun.resize(NUM_LEG);
    gen_dvdphi_fun.resize(NUM_LEG);
    gen_dvddphi_fun.resize(NUM_LEG);
    gen_dvdw_fun.resize(NUM_LEG);
    gen_dvdwf_fun.resize(NUM_LEG);
    gen_dvdbg_fun.resize(NUM_LEG);
    gen_dvdbf_fun.resize(NUM_LEG);
    gen_dvdbv_fun.resize(NUM_LEG);
    gen_dvdd0_fun.resize(NUM_LEG);
    for (int i = 0; i < NUM_LEG; i++) {
      std::string func_name = fun_name_list[0] + std::to_string(i);
      gen_v_fun[i] = casadi::external(func_name, code_gen_path + func_name + ".so");
      func_name = fun_name_list[1] + std::to_string(i);
      gen_dvdphi_fun[i] = casadi::external(func_name, code_gen_path + func_name + ".so");
      func_name = fun_name_list[2] + std::to_string(i);
      gen_dvddphi_fun[i] = casadi::external(func_name, code_gen_path + func_name + ".so");
      func_name = fun_name_list[3] + std::to_string(i);
      gen_dvdw_fun[i] = casadi::external(func_name, code_gen_path + func_name + ".so");
      func_name = fun_name_list[4] + std::to_string(i);
      gen_dvdwf_fun[i] = casadi::external(func_name, code_gen_path + func_name + ".so");
      func_name = fun_name_list[5] + std::to_string(i);
      gen_dvdbg_fun[i] = casadi::external(func_name, code_gen_path + func_name + ".so");
      func_name = fun_name_list[6] + std::to_string(i);
      gen_dvdbf_fun[i] = casadi::external(func_name, code_gen_path + func_name + ".so");
      func_name = fun_name_list[7] + std::to_string(i);
      gen_dvdbv_fun[i] = casadi::external(func_name, code_gen_path + func_name + ".so");
      func_name = fun_name_list[8] + std::to_string(i);
      gen_dvdd0_fun[i] = casadi::external(func_name, code_gen_path + func_name + ".so");
    }

    // setupp arg
    arg.clear();
    casadi::DM jang_dm = casadi::DM::zeros(3, 1);
    casadi::DM jvel_dm = casadi::DM::zeros(3, 1);
    casadi::DM body_gyr_dm = casadi::DM::zeros(3, 1);
    casadi::DM foot_gyr_dm = casadi::DM::zeros(3, 1);
    casadi::DM linearized_bg_dm = casadi::DM::zeros(3, 1);
    casadi::DM linearized_bf_dm = casadi::DM::zeros(3, 1);
    casadi::DM linearized_bv_dm = casadi::DM::zeros(3, 1);
    casadi::DM d0_dm = casadi::DM::zeros(1, 1);
    casadi::DM kin_dm = casadi::DM::zeros(5, NUM_LEG);
    casadi::DM R_fi_dm = casadi::DM::zeros(3, 3);

    // arg = {jang_dm, jvel_dm, body_gyr_dm, linearized_bg_dm, foot_gyr_dm, linearized_bf_dm, kin_dm, R_fi_dm, d0_dm, linearized_bv_dm};
    arg.push_back(jang_dm);
    arg.push_back(jvel_dm);
    arg.push_back(body_gyr_dm);
    arg.push_back(linearized_bg_dm);
    arg.push_back(foot_gyr_dm);
    arg.push_back(linearized_bf_dm);
    arg.push_back(kin_dm);
    arg.push_back(R_fi_dm);
    arg.push_back(d0_dm);
    arg.push_back(linearized_bv_dm);
  }

  // codegen function
  void codegen(casadi::Function func) {
    std::string name = func.name();

    func.generate(name, casadi::Dict{{"mex", false}, {"with_header", false}, {"cpp", false}});
    // move file to the same location
    std::string mv_command = "mv " + name + ".c " + code_gen_path;
    int mv_flag = system(mv_command.c_str());
    mv_command = "mv " + name + ".h " + code_gen_path;
    mv_flag = system(mv_command.c_str());
    std::string compile_command = "gcc -fPIC -shared -O3 " + code_gen_path + name + ".c -o " + code_gen_path + name + ".so";
    int flag = system(compile_command.c_str());
  }

  bool if_code_gen() {
    // check if the shared lib exists in the code_gen_path
    std::string filename = code_gen_path + fun_name_list[0] + std::to_string(0) + ".so";
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
  }

  // helper function, convert Eigen arguments into casadi::DM array
  void eig_args_asm(const int leg_id, const Eigen::Vector3d& jang, const Eigen::Vector3d& jvel, const Eigen::Vector3d& body_gyr,
                    const Eigen::Vector3d& foot_gyr, const Eigen::Vector3d& linearized_bg, const Eigen::Vector3d& linearized_bf,
                    const Eigen::Vector3d& linearized_bv, const double d0) {
    std::copy(jang.data(), jang.data() + jang.size(), arg[0].ptr());
    std::copy(jvel.data(), jvel.data() + jvel.size(), arg[1].ptr());
    std::copy(body_gyr.data(), body_gyr.data() + body_gyr.size(), arg[2].ptr());
    std::copy(linearized_bg.data(), linearized_bg.data() + linearized_bg.size(), arg[3].ptr());
    std::copy(foot_gyr.data(), foot_gyr.data() + foot_gyr.size(), arg[4].ptr());
    std::copy(linearized_bf.data(), linearized_bf.data() + linearized_bf.size(), arg[5].ptr());
    std::copy(kin_true_.data(), kin_true_.data() + kin_true_.size(), arg[6].ptr());
    Eigen::Matrix3d tmp = mipo_utils.R_fi_list[leg_id];
    std::copy(tmp.data(), tmp.data() + tmp.size(), arg[7].ptr());
    arg[8](0, 0) = d0;
    std::copy(linearized_bv.data(), linearized_bv.data() + linearized_bv.size(), arg[9].ptr());

    return;
  }

  // get body velocity from leg joint angle and velocity, and foot gyro, the body velocity is expressed in body frame
  // core function, everything else in this file depends on this
  Eigen::Matrix<casadi::SX, 3, 1> getBodyVel(int leg_id, const Eigen::Matrix<casadi::SX, 3, 1>& j_ang,
                                             const Eigen::Matrix<casadi::SX, 3, 1>& j_vel, const Eigen::Matrix<casadi::SX, 3, 1>& body_w,
                                             const Eigen::Matrix<casadi::SX, 3, 1>& bg, const Eigen::Matrix<casadi::SX, 3, 1>& foot_w,
                                             const Eigen::Matrix<casadi::SX, 3, 1>& bf,
                                             const Eigen::Matrix<casadi::SX, Eigen::Dynamic, NUM_LEG> kin_param,
                                             const Eigen::Matrix<casadi::SX, 3, 3>& R_fi, const Eigen::Matrix<casadi::SX, 1, 1>& d0,
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
    Eigen::Matrix<casadi::SX, 3, 1> foot_w_body = R_bf * R_fi * foot_w_no_bias;
    Eigen::Matrix<casadi::SX, 3, 1> foot_support_vec = -p_rf / p_rf.norm() * d0;
    Eigen::Matrix<casadi::SX, 3, 1> foot_v_pivoting = legged::skewSymmetric<casadi::SX>(foot_w_body) * foot_support_vec;

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
  std::vector<casadi::Function> v_fun;
  std::vector<casadi::Function> dvdphi_fun;
  std::vector<casadi::Function> dvddphi_fun;
  std::vector<casadi::Function> dvdw_fun;
  std::vector<casadi::Function> dvdwf_fun;

  std::vector<casadi::Function> dvdbg_fun;
  std::vector<casadi::Function> dvdbf_fun;
  std::vector<casadi::Function> dvdbv_fun;
  std::vector<casadi::Function> dvdd0_fun;

  std::vector<casadi::Function> gen_v_fun;
  std::vector<casadi::Function> gen_dvdphi_fun;
  std::vector<casadi::Function> gen_dvddphi_fun;
  std::vector<casadi::Function> gen_dvdw_fun;
  std::vector<casadi::Function> gen_dvdwf_fun;

  std::vector<casadi::Function> gen_dvdbg_fun;
  std::vector<casadi::Function> gen_dvdbf_fun;
  std::vector<casadi::Function> gen_dvdbv_fun;
  std::vector<casadi::Function> gen_dvdd0_fun;

  // arg helper
  std::vector<casadi::DM> arg;
  std::mutex mtx;

  // code gen
  const std::string code_gen_path = "/home/EstimationUser/estimation_ws/src/cerberus2/code_gen/";

  // a list of string names: "v_fun", "dvdphi_fun", "dvddphi_fun", "dvdw_fun", "dvdwf_fun", "dvdbg_fun", "dvdbf_fun", "dvdbv_fun",
  // "dvdd0_fun"
  const std::vector<std::string> fun_name_list = {"v_fun",     "dvdphi_fun", "dvddphi_fun", "dvdw_fun", "dvdwf_fun",
                                                  "dvdbg_fun", "dvdbf_fun",  "dvdbv_fun",   "dvdd0_fun"};
};