#include <Eigen/SparseQR>

#include "mipo/MIPOEstimator.hpp"
#include "utils/casadi_kino.hpp"
#include "utils/utils.hpp"

void MIPOEstimatorSensorData::footIMUToBody() {
  for (int i = 0; i < NUM_LEG; i++) {
    Eigen::Vector3d leg_ang = joint_angles.segment<3>(i * 3);
    Eigen::Matrix3d R_bf = legged::fk_pf_rot(leg_ang);

    foot_acc_body.col(i) = R_bf * R_fi_list[i] * foot_acc.col(i);
    foot_gyro_body.col(i) = R_bf * R_fi_list[i] * foot_gyro.col(i);
  }
}

void MIPOEstimatorSensorData::loadFromVec(Eigen::Matrix<double, 55, 1> sensor_vec) {
  body_gyro = sensor_vec.segment<3>(0);
  body_acc = sensor_vec.segment<3>(3);
  joint_angles = sensor_vec.segment<12>(6);
  joint_velocities = sensor_vec.segment<12>(18);
  for (int i = 0; i < NUM_LEG; i++) {
    foot_acc.col(i) = sensor_vec.segment<3>(30 + i * 3);
    foot_gyro.col(i) = sensor_vec.segment<3>(3 + 3 + 12 + 12 + 12 + i * 3);
  }
  body_yaw = sensor_vec(54);
  footIMUToBody();
}

MIPOEstimator::MIPOEstimator() {
  mipo_use_foot_ang_contact_model_ = true;
  rho_true_ = Eigen::Matrix<double, 5, NUM_LEG>::Zero();
  // col 0: FL
  //   rho_true_(0, 0) = 0.1805;
  //   rho_true_(1, 0) = 0.047;
  //   rho_true_(2, 0) = 0.0838;
  //   rho_true_(3, 0) = 0.2;
  //   rho_true_(4, 0) = 0.2;
  //   // col 1: FR
  //   rho_true_(0, 1) = 0.1805;
  //   rho_true_(1, 1) = -0.047;
  //   rho_true_(2, 1) = -0.0838;
  //   rho_true_(3, 1) = 0.2;
  //   rho_true_(4, 1) = 0.2;
  //   // col 2: RL
  //   rho_true_(0, 2) = -0.1805;
  //   rho_true_(1, 2) = 0.047;
  //   rho_true_(2, 2) = 0.0838;
  //   rho_true_(3, 2) = 0.2;
  //   rho_true_(4, 2) = 0.2;
  //   // col 3: RR
  //   rho_true_(0, 3) = -0.1805;
  //   rho_true_(1, 3) = -0.047;
  //   rho_true_(2, 3) = -0.0838;
  //   rho_true_(3, 3) = 0.2;
  //   rho_true_(4, 3) = 0.2;
  rho_true_ << 0.1805, 0.1805, -0.1805, -0.1805, 0.047, -0.047, 0.047, -0.047, 0.0838, -0.0838, 0.0838, -0.0838, 0.2, 0.2, 0.2, 0.2, 0.2,
      0.2, 0.2, 0.2;

  // must be called every time at the beginning
  mipo_init_casadi();
  mipo_init_noise();
}

MIPOEstimator::~MIPOEstimator() {}

void MIPOEstimator::ekfUpdate(const Eigen::Matrix<double, MS_SIZE, 1>& x_k, const Eigen::Matrix<double, MS_SIZE, MS_SIZE>& P_k,
                              const MIPOEstimatorSensorData& sensor_data_k, const MIPOEstimatorSensorData& sensor_data_k1, const double dt,
                              // output
                              Eigen::Matrix<double, MS_SIZE, 1>& x_k1, Eigen::Matrix<double, MS_SIZE, MS_SIZE>& P_k1,
                              Eigen::Matrix<double, NUM_LEG, 1>& contact_est) {
  // process sensor data into input and measurement
  // caution: make sure these two functions are called by outside caller
  // sensor_data_k.footIMUToBody();
  // sensor_data_k1.footIMUToBody();

  Eigen::Matrix<double, MI_SIZE, 1> u_k;
  u_k.segment<3>(0) = sensor_data_k.body_gyro;  // notice the order, defined in mipo_process_dyn_casadi
  u_k.segment<3>(3) = sensor_data_k.body_acc;
  for (int i = 0; i < NUM_LEG; i++) {
    u_k.segment<3>(6 + 3 * i) = sensor_data_k.foot_acc_body.col(i);
  }
  u_k(MI_SIZE - 1) = dt;
  Eigen::Matrix<double, MI_SIZE, 1> u_k1;
  u_k1.segment<3>(0) = sensor_data_k1.body_gyro;  // notice the order, defined in mipo_process_dyn_casadi
  u_k1.segment<3>(3) = sensor_data_k1.body_acc;
  for (int i = 0; i < NUM_LEG; i++) {
    u_k1.segment<3>(6 + 3 * i) = sensor_data_k1.foot_acc_body.col(i);
  }
  u_k1(MI_SIZE - 1) = dt;
  // check uk and uk1
  //   std::cout << "u_k: " << u_k.transpose() << std::endl;
  //   std::cout << "u_k1: " << u_k1.transpose() << std::endl;

  //   // assemble measurement vector z
  Eigen::Matrix<double, MZ_SIZE, 1> z_k;
  z_k.segment<3>(0) = sensor_data_k.body_gyro;
  z_k.segment<NUM_DOF>(3) = sensor_data_k.joint_angles;
  z_k.segment<NUM_DOF>(3 + NUM_DOF) = sensor_data_k.joint_velocities;
  z_k(3 + 2 * NUM_DOF) = sensor_data_k.body_yaw;
  // fill z_k.segment<NUM_DOF>(3 + 2 * NUM_DOF + 1)
  for (int i = 0; i < NUM_LEG; i++) {
    z_k.segment<3>(3 + 2 * NUM_DOF + 1 + 3 * i) = sensor_data_k.foot_gyro_body.col(i);
  }
  // check z_k
  //   std::cout << "z_k: " << z_k.transpose() << std::endl;

  //   // EKF steps
  //   // 1. predict

  Eigen::Matrix<double, MS_SIZE, 1> x_k1_pred = proc_func(x_k, u_k, u_k1, dt);
  //   std::cout << "x_k1_pred: " << x_k1_pred.transpose() << std::endl;
  Eigen::Matrix<double, MS_SIZE, MS_SIZE> F = proc_func_x_jac(x_k, u_k, u_k1, dt);
  Eigen::Matrix<double, MS_SIZE, MI_SIZE> G = proc_func_u_jac(x_k, u_k, u_k1, dt);
  // Eigen::SparseMatrix<double> sparse_P_k = P_k.sparseView();
  Eigen::Matrix<double, MS_SIZE, MS_SIZE> P_k_pred = F * P_k * F.transpose();
  for (int i = 0; i < MS_SIZE; i++) {
    P_k_pred(i, i) += dt * Q1.diagonal()(i);
  }  // do this to prevent a strange bug mum map invalid pointer

  Eigen::Matrix<double, MS_SIZE, MS_SIZE> GQ2Gt = G * Q2 * G.transpose() * dt;
  P_k_pred += GQ2Gt;

  //   std::cout << "P_k_pred: " << P_k_pred.diagonal().transpose() <<
  //   std::endl;
  // 2. update
  Eigen::Matrix<double, MY_SIZE, 1> y_k = meas_func(x_k1_pred, z_k);
  //   std::cout << "y_k: " << y_k.transpose() << std::endl;
  Eigen::Matrix<double, MY_SIZE, MS_SIZE> H = meas_func_jac(x_k1_pred, z_k);
  //   std::cout << "H: " << H << std::endl;
  Eigen::Matrix<double, MY_SIZE, MY_SIZE> S = H * P_k_pred * H.transpose();
  for (int i = 0; i < MY_SIZE; i++) {
    S(i, i) += R.diagonal()(i);
  }  // do this to prevent a strange bug mum map invalid pointer

  // S is different from that in MATLAB
  // print the diagonal of R and S
  //   std::cout << "R: " << R.diagonal().transpose() << std::endl;
  //   std::cout << "S: " << S.diagonal().transpose() << std::endl;

  // 2.1 outlier detection
  // create a mask for outlier detection, the size is MY_SIZE, the content
  //   is 0
  // or 1, 0 means outlier, 1 means inlier. The mask is initialized as
  //   all 1.
  contact_est.setOnes();
  Eigen::Matrix<int, MY_SIZE, 1> outlier_mask;
  outlier_mask.setOnes();
  for (int i = 0; i < NUM_LEG; i++) {
    // if the measurement at MY_PER_LEG*i+6 to MY_PER_LEG*i+8 has a
    // mahalonobinous distance larger than a certain value, then it is an
    // outlier
    Eigen::Vector3d seg_mes = y_k.segment<3>(MY_PER_LEG * i + 6);
    Eigen::Matrix3d seg_S = S.block<3, 3>(MY_PER_LEG * i + 6, MY_PER_LEG * i + 6);
    double mahal_dist = sqrt(seg_mes.transpose() * seg_S.inverse() * seg_mes);
    // std::cout << "mahal_dist: " << mahal_dist << std::endl;
    if (mahal_dist > 1.0) {
      contact_est(i) = 0;
      outlier_mask.segment<3>(MY_PER_LEG * i + 6).setZero();
      // outlier_mask(MY_PER_LEG * i + 9) = 0.0;
    }
    // TODO output this as contact flag
  }
  Eigen::SparseMatrix<double> mask_sparse(outlier_mask.size(), outlier_mask.sum());
  int col_idx = 0;
  for (int i = 0; i < outlier_mask.size(); i++) {
    if (outlier_mask(i) == 1) {
      mask_sparse.insert(i, col_idx) = 1;
      col_idx++;
    }
  }
  // use the mask to extract rows of H and y_k, and extract rows/columns of
  //   S that correspond to inliers
  // convert the mask to a selection matrix whose size is MY_SIZE*the
  //   number of ones in the mask
  Eigen::VectorXd y_k_masked = mask_sparse.transpose() * y_k;
  Eigen::MatrixXd H_masked = mask_sparse.transpose() * H;
  Eigen::MatrixXd S_masked = mask_sparse.transpose() * S * mask_sparse;

  // 2.2 update, using colhouseholderqr to solve the linear system
  // Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>>
  //     sparse_solver;
  // sparse_solver.compute(S_masked);
  Eigen::VectorXd Sy = S_masked.ldlt().solve(y_k_masked);

  Eigen::MatrixXd tmp = P_k_pred * H_masked.transpose();
  Eigen::VectorXd update = tmp * Sy;
  //   std::cout << "update: " << update.transpose() << std::endl;
  x_k1 = x_k1_pred - update;

  P_k1 = P_k_pred - tmp * S_masked.ldlt().solve(H_masked) * P_k_pred;

  // regularize P_k1 to make it symmetric
  P_k1 = (P_k1 + P_k1.transpose()) / 2.0;
  // // reduce position drift ( do we need this)
  // if (P_k1.block<2, 2>(0, 0).determinant() > 1e-6) {
  //   P_k1.block<2, MY_SIZE>(0, 2).setZero();
  //   P_k1.block<MY_SIZE, 2>(2, 0).setZero();
  //   P_k1.block<2, 2>(0, 0) /= 10.0;
  // }
  return;
}

Eigen::Matrix<double, MS_SIZE, 1> MIPOEstimator::ekfInitState(const MIPOEstimatorSensorData& sensor_data_k) {
  Eigen::Matrix<double, MS_SIZE, 1> x_k;
  x_k.setZero();
  // initial pos
  Eigen::Vector3d init_pos;
  init_pos.setZero();
  init_pos(2) = 0.32;  // TODO: change this to a parameter
  x_k.segment<3>(0) = init_pos;
  x_k.segment<3>(3) = Eigen::Vector3d::Zero();  // init vel
  // initial euler
  Eigen::Vector3d init_euler;
  init_euler.setZero();
  x_k.segment<3>(6) = init_euler;
  Eigen::Matrix3d R_er = legged::euler_to_rot(init_euler);
  // from joint angles calculate foot pos
  for (int i = 0; i < NUM_LEG; i++) {
    Eigen::Vector3d joint_ang = sensor_data_k.joint_angles.segment<3>(i * 3);
    Eigen::Matrix<double, -1, 1> param_leg = rho_true_.col(i);
    Eigen::Vector3d foot_pos = legged::fk_pf_pos(joint_ang, param_leg);

    x_k.segment<3>(9 + i * 6) = R_er * foot_pos + init_pos;
  }
  return x_k;
}

/*
 * following interface functions are called by the external user
 */
// input and output are Eigen Vector or sparse Matrix

Eigen::Matrix<double, MS_SIZE, 1> MIPOEstimator::proc_func(const Eigen::Matrix<double, MS_SIZE, 1> x,
                                                           const Eigen::Matrix<double, MI_SIZE, 1> u, double dt) {
  return mipo_xk1_eigen(x, u, u, dt);
}

Eigen::Matrix<double, MS_SIZE, 1> MIPOEstimator::proc_func(const Eigen::Matrix<double, MS_SIZE, 1> x,
                                                           const Eigen::Matrix<double, MI_SIZE, 1> u0,
                                                           const Eigen::Matrix<double, MI_SIZE, 1> u1, double dt) {
  return mipo_xk1_eigen(x, u0, u1, dt);
}

Eigen::Matrix<double, MS_SIZE, MS_SIZE> MIPOEstimator::proc_func_x_jac(const Eigen::Matrix<double, MS_SIZE, 1> x,
                                                                       const Eigen::Matrix<double, MI_SIZE, 1> u, double dt) {
  std::lock_guard<std::mutex> lock(casadi_mtx);
  // convert x and u to casadi DM type
  casadi::DM x_dm = Utils::eig_to_cas_DM<MS_SIZE, 1>(x);
  casadi::DM u_dm = Utils::eig_to_cas_DM<MI_SIZE, 1>(u);
  // covert rho to casadi DM type
  casadi::DM dt_dm = casadi::DM(dt);
  std::vector<casadi::DM> arg = {x_dm, u_dm, u_dm, dt_dm};
  std::vector<casadi::DM> res = mipo_process_dyn_jac_x_func_(arg);
  // convert the result to eigen matrix
  Eigen::Matrix<double, MS_SIZE, MS_SIZE> SpMatrx = Utils::cas_DM_to_eig_mat<MS_SIZE, MS_SIZE>(res[0]);

  return SpMatrx;
}

Eigen::Matrix<double, MS_SIZE, MS_SIZE> MIPOEstimator::proc_func_x_jac(const Eigen::Matrix<double, MS_SIZE, 1> x,
                                                                       const Eigen::Matrix<double, MI_SIZE, 1> u0,
                                                                       const Eigen::Matrix<double, MI_SIZE, 1> u1, double dt) {
  std::lock_guard<std::mutex> lock(casadi_mtx);
  // convert x and u to casadi DM type
  casadi::DM x_dm = Utils::eig_to_cas_DM<MS_SIZE, 1>(x);
  casadi::DM u0_dm = Utils::eig_to_cas_DM<MI_SIZE, 1>(u0);
  casadi::DM u1_dm = Utils::eig_to_cas_DM<MI_SIZE, 1>(u1);
  // covert rho to casadi DM type
  casadi::DM dt_dm = casadi::DM(dt);
  std::vector<casadi::DM> arg = {x_dm, u0_dm, u1_dm, dt_dm};
  std::vector<casadi::DM> res = mipo_process_dyn_jac_x_func_(arg);
  // convert the result to eigen matrix
  Eigen::Matrix<double, MS_SIZE, MS_SIZE> SpMatrx = Utils::cas_DM_to_eig_mat<MS_SIZE, MS_SIZE>(res[0]);

  return SpMatrx;
}

Eigen::Matrix<double, MS_SIZE, MI_SIZE> MIPOEstimator::proc_func_u_jac(const Eigen::Matrix<double, MS_SIZE, 1> x,
                                                                       const Eigen::Matrix<double, MI_SIZE, 1> u, double dt) {
  std::lock_guard<std::mutex> lock(casadi_mtx);
  // convert x and u to casadi DM type
  casadi::DM x_dm = Utils::eig_to_cas_DM<MS_SIZE, 1>(x);
  casadi::DM u_dm = Utils::eig_to_cas_DM<MI_SIZE, 1>(u);
  // covert rho to casadi DM type
  casadi::DM dt_dm = casadi::DM(dt);
  std::vector<casadi::DM> arg = {x_dm, u_dm, u_dm, dt_dm};
  std::vector<casadi::DM> res = mipo_process_dyn_jac_u_func_(arg);
  // convert the result to eigen matrix
  Eigen::Matrix<double, MS_SIZE, MI_SIZE> SpMatrx = Utils::cas_DM_to_eig_mat<MS_SIZE, MI_SIZE>(res[0]);

  return SpMatrx;
}

Eigen::Matrix<double, MS_SIZE, MI_SIZE> MIPOEstimator::proc_func_u_jac(const Eigen::Matrix<double, MS_SIZE, 1> x,
                                                                       const Eigen::Matrix<double, MI_SIZE, 1> u0,
                                                                       const Eigen::Matrix<double, MI_SIZE, 1> u1, double dt) {
  std::lock_guard<std::mutex> lock(casadi_mtx);
  // convert x and u to casadi DM type
  casadi::DM x_dm = Utils::eig_to_cas_DM<MS_SIZE, 1>(x);
  casadi::DM u0_dm = Utils::eig_to_cas_DM<MI_SIZE, 1>(u0);
  casadi::DM u1_dm = Utils::eig_to_cas_DM<MI_SIZE, 1>(u1);
  // covert rho to casadi DM type
  casadi::DM dt_dm = casadi::DM(dt);
  std::vector<casadi::DM> arg = {x_dm, u0_dm, u1_dm, dt_dm};
  std::vector<casadi::DM> res = mipo_process_dyn_jac_u_func_(arg);
  // convert the result to eigen matrix
  Eigen::Matrix<double, MS_SIZE, MI_SIZE> SpMatrx = Utils::cas_DM_to_eig_mat<MS_SIZE, MI_SIZE>(res[0]);

  return SpMatrx;
}

// measurement function can directly use the template funtion
Eigen::Matrix<double, MY_SIZE, 1> MIPOEstimator::meas_func(const Eigen::Matrix<double, MS_SIZE, 1> x,
                                                           const Eigen::Matrix<double, MZ_SIZE, 1> z) {
  return mipo_measurement_casadi<double>(x, z, rho_true_);
}

// meausrement jacobian must call the casadi function
// but the casadi function can be generated first
Eigen::Matrix<double, MY_SIZE, MS_SIZE> MIPOEstimator::meas_func_jac(const Eigen::Matrix<double, MS_SIZE, 1> x,
                                                                     const Eigen::Matrix<double, MZ_SIZE, 1> z) {
  std::lock_guard<std::mutex> lock(casadi_mtx);
  // convert x and z to casadi DM type
  casadi::DM x_dm = Utils::eig_to_cas_DM<MS_SIZE, 1>(x);
  casadi::DM z_dm = Utils::eig_to_cas_DM<MZ_SIZE, 1>(z);
  // covert rho to casadi DM type
  casadi::DM rho_dm = Utils::eig_to_cas_DM<5, NUM_LEG>(rho_true_);
  std::vector<casadi::DM> arg = {x_dm, z_dm, rho_dm};
  std::vector<casadi::DM> res = mipo_measurement_jac_func_(arg);
  // convert the result to eigen matrix
  Eigen::Matrix<double, MY_SIZE, MS_SIZE> SpMatrx = Utils::cas_DM_to_eig_mat<MY_SIZE, MS_SIZE>(res[0]);

  return SpMatrx;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, MS_SIZE, 1> MIPOEstimator::mipo_process_dyn_casadi(const Eigen::Matrix<SCALAR_T, MS_SIZE, 1> x,
                                                                           const Eigen::Matrix<SCALAR_T, 19, 1> u) {
  Eigen::Matrix<SCALAR_T, 3, 1> pos = x.segment(0, 3);
  Eigen::Matrix<SCALAR_T, 3, 1> vel = x.segment(3, 3);
  Eigen::Matrix<SCALAR_T, 3, 1> euler = x.segment(6, 3);

  Eigen::Matrix<SCALAR_T, 3, 1> foot1_pos = x.segment(9, 3);
  Eigen::Matrix<SCALAR_T, 3, 1> foot1_vel = x.segment(12, 3);
  Eigen::Matrix<SCALAR_T, 3, 1> foot2_pos = x.segment(15, 3);
  Eigen::Matrix<SCALAR_T, 3, 1> foot2_vel = x.segment(18, 3);
  Eigen::Matrix<SCALAR_T, 3, 1> foot3_pos = x.segment(21, 3);
  Eigen::Matrix<SCALAR_T, 3, 1> foot3_vel = x.segment(24, 3);
  Eigen::Matrix<SCALAR_T, 3, 1> foot4_pos = x.segment(27, 3);
  Eigen::Matrix<SCALAR_T, 3, 1> foot4_vel = x.segment(30, 3);

  Eigen::Matrix<SCALAR_T, 3, 1> ba = x.segment(33, 3);  // body acc bias
  Eigen::Matrix<SCALAR_T, 3, 1> bg = x.segment(36, 3);  // gyro bias

  Eigen::Matrix<SCALAR_T, 3, 1> foot1_ba = x.segment(39, 3);  // foot 1 acc bias
  Eigen::Matrix<SCALAR_T, 3, 1> foot2_ba = x.segment(42, 3);  // foot 2 acc bias
  Eigen::Matrix<SCALAR_T, 3, 1> foot3_ba = x.segment(45, 3);  // foot 3 acc bias
  Eigen::Matrix<SCALAR_T, 3, 1> foot4_ba = x.segment(48, 3);  // foot 4 acc bias

  Eigen::Matrix<SCALAR_T, 3, 1> w = u.segment(0, 3) - bg;  // body angular velocity
  Eigen::Matrix<SCALAR_T, 3, 1> a = u.segment(3, 3) - ba;  // body linear acceleration

  Eigen::Matrix<SCALAR_T, 3, 1> foot1_acc = u.segment(6, 3) - foot1_ba;
  Eigen::Matrix<SCALAR_T, 3, 1> foot2_acc = u.segment(9, 3) - foot2_ba;
  Eigen::Matrix<SCALAR_T, 3, 1> foot3_acc = u.segment(12, 3) - foot3_ba;
  Eigen::Matrix<SCALAR_T, 3, 1> foot4_acc = u.segment(15, 3) - foot4_ba;

  Eigen::Matrix<SCALAR_T, 3, 1> deuler = legged::mtx_w_to_euler_dot<SCALAR_T>(euler) * w;

  Eigen::Matrix<SCALAR_T, 3, 3> R = legged::euler_to_rot<SCALAR_T>(euler);

  Eigen::Matrix<SCALAR_T, 3, 1> gravity(0, 0, 9.81);
  Eigen::Matrix<SCALAR_T, 3, 1> acc = R * a - gravity;

  Eigen::Matrix<SCALAR_T, 3, 1> foot1_acc_w = R * foot1_acc - gravity;
  Eigen::Matrix<SCALAR_T, 3, 1> foot2_acc_w = R * foot2_acc - gravity;
  Eigen::Matrix<SCALAR_T, 3, 1> foot3_acc_w = R * foot3_acc - gravity;
  Eigen::Matrix<SCALAR_T, 3, 1> foot4_acc_w = R * foot4_acc - gravity;

  Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> xdot(MS_SIZE);
  xdot << vel, acc, deuler, foot1_vel, foot1_acc_w, foot2_vel, foot2_acc_w, foot3_vel, foot3_acc_w, foot4_vel, foot4_acc_w,
      Eigen::Matrix<SCALAR_T, 3, 1>::Zero(),  // body acc bias
      Eigen::Matrix<SCALAR_T, 3, 1>::Zero(),  // gyro bias
      Eigen::Matrix<SCALAR_T, 3, 1>::Zero(),  // foot 1 acc bias
      Eigen::Matrix<SCALAR_T, 3, 1>::Zero(),  // foot 2 acc bias
      Eigen::Matrix<SCALAR_T, 3, 1>::Zero(),  // foot 3 acc bias
      Eigen::Matrix<SCALAR_T, 3, 1>::Zero(),  // foot 4 acc bias
      1;

  return xdot;
}

/* bug 03-20: test_kino.cpp:(.text+0xa946): undefined reference to
 * `Eigen::Matrix<double, MS_SIZE, 1, 0, MS_SIZE, 1>
 * MIPOEstimator::mipo_xk1_casadi<double>(Eigen::Matrix<double, MS_SIZE, 1, 0,
 * MS_SIZE, 1>, Eigen::Matrix<double, 19, 1, 0, 19, 1>, Eigen::Matrix<double,
 * 19, 1, 0, 19, 1>, double)'*/
Eigen::Matrix<casadi::SX, MS_SIZE, 1> MIPOEstimator::mipo_xk1_casadi(const Eigen::Matrix<casadi::SX, MS_SIZE, 1> x,
                                                                     const Eigen::Matrix<casadi::SX, MI_SIZE, 1> u0,
                                                                     const Eigen::Matrix<casadi::SX, MI_SIZE, 1> u1, casadi::SX dt) {
  // use rk4 function to get xk+1
  // pass MIPOEstimator::mipo_process_dyn_casadi as a std::function pointer
  Eigen::Matrix<casadi::SX, MS_SIZE, 1> x_k1 = legged::dyn_rk4_casadi<casadi::SX, MS_SIZE, MI_SIZE>(
      x, u0, u1, dt, std::bind(&MIPOEstimator::mipo_process_dyn_casadi<casadi::SX>, this, std::placeholders::_1, std::placeholders::_2));

  return x_k1;
}

Eigen::Matrix<double, MS_SIZE, 1> MIPOEstimator::mipo_xk1_eigen(const Eigen::Matrix<double, MS_SIZE, 1> x,
                                                                const Eigen::Matrix<double, MI_SIZE, 1> u0,
                                                                const Eigen::Matrix<double, MI_SIZE, 1> u1, double dt) {
  // use rk4 function to get xk+1
  // pass MIPOEstimator::mipo_process_dyn_casadi as a std::function pointer
  Eigen::Matrix<double, MS_SIZE, 1> x_k1 = legged::dyn_rk4_casadi<double, MS_SIZE, MI_SIZE>(
      x, u0, u1, dt, std::bind(&MIPOEstimator::mipo_process_dyn_casadi<double>, this, std::placeholders::_1, std::placeholders::_2));

  return x_k1;
}

// measurement functions - eigen/casadi
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, MY_SIZE, 1> MIPOEstimator::mipo_measurement_casadi(const Eigen::Matrix<SCALAR_T, MS_SIZE, 1> x,
                                                                           const Eigen::Matrix<SCALAR_T, MZ_SIZE, 1> z,
                                                                           const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, NUM_LEG> param) {
  Eigen::Matrix<SCALAR_T, 3, 1> wk = z.segment(0, 3);
  Eigen::Matrix<SCALAR_T, 12, 1> phik = z.segment(3, 12);
  Eigen::Matrix<SCALAR_T, 12, 1> dphik = z.segment(12 + 3, 12);
  SCALAR_T yawk = z(12 + 3 + 12);
  Eigen::Matrix<SCALAR_T, 12, 1> foot_gyrok = z.segment(12 + 3 + 12 + 1, 12);

  Eigen::Matrix<SCALAR_T, 3, 1> pos = x.segment(0, 3);
  Eigen::Matrix<SCALAR_T, 3, 1> vel = x.segment(3, 3);
  Eigen::Matrix<SCALAR_T, 3, 1> euler = x.segment(6, 3);
  Eigen::Matrix<SCALAR_T, 3, 3> R_er = legged::euler_to_rot<SCALAR_T>(euler);
  Eigen::Matrix<SCALAR_T, 12, 1> foot_pos;
  foot_pos << x.segment(9, 3), x.segment(15, 3), x.segment(21, 3), x.segment(27, 3);
  Eigen::Matrix<SCALAR_T, 12, 1> foot_vel;
  foot_vel << x.segment(12, 3), x.segment(18, 3), x.segment(24, 3), x.segment(30, 3);
  Eigen::Matrix<SCALAR_T, 3, 1> bg = x.segment(36, 3);

  Eigen::Matrix<SCALAR_T, MY_SIZE, 1> meas_residual;
  meas_residual.setZero();
  for (int i = 0; i < NUM_LEG; ++i) {
    Eigen::Matrix<SCALAR_T, 3, 1> angle = phik.segment(i * 3, 3);
    Eigen::Matrix<SCALAR_T, 3, 1> av = dphik.segment(i * 3, 3);
    Eigen::Matrix<SCALAR_T, -1, 1> param_leg = param.col(i);
    Eigen::Matrix<SCALAR_T, 3, 1> p_rf = legged::fk_pf_pos(angle, param_leg);
    Eigen::Matrix<SCALAR_T, 3, 3> J_rf = legged::d_fk_dt(angle, param_leg);
    Eigen::Matrix<SCALAR_T, 3, 1> w_k_no_bias = wk - bg;
    Eigen::Matrix<SCALAR_T, 3, 1> leg_v = (J_rf * av + legged::skewSymmetric<SCALAR_T>(w_k_no_bias) * p_rf);

    meas_residual.segment(i * MY_PER_LEG, 3) = p_rf - R_er.transpose() * (foot_pos.segment(i * 3, 3) - pos);
    meas_residual.segment(i * MY_PER_LEG + 3, 3) = foot_vel.segment(i * 3, 3) - (vel + R_er * leg_v);

    Eigen::Matrix<SCALAR_T, 3, 1> foot_w_robot = foot_gyrok.segment(i * 3, 3);
    Eigen::Matrix<SCALAR_T, 3, 1> foot_w_world = R_er * foot_w_robot;
    Eigen::Matrix<SCALAR_T, 3, 1> p_rf_world = R_er * p_rf;

    Eigen::Matrix<SCALAR_T, 3, 1> foot_support_vec = -p_rf_world / p_rf_world.norm() * 0.05;  // distance from contact surface to foot IMU
    Eigen::Matrix<SCALAR_T, 3, 1> foot_vel_world = foot_w_world.cross(foot_support_vec);

    if (mipo_use_foot_ang_contact_model_ == true) {
      meas_residual.segment(i * MY_PER_LEG + 6, 3) = foot_vel.segment(i * 3, 3) - foot_vel_world;
    } else {
      meas_residual.segment(i * MY_PER_LEG + 6, 3) = foot_vel.segment(i * 3, 3);
    }

    meas_residual(i * MY_PER_LEG + 9) = foot_pos(i * 3 + 2);
  }

  meas_residual(MY_SIZE - 1) = yawk - euler(2);

  return meas_residual;
}

void MIPOEstimator::mipo_init_casadi() {
  // define casadi symbols for state, input, parameters, sensor data, and
  // parameters
  casadi::SX x = casadi::SX::sym("x", MS_SIZE);
  casadi::SX z = casadi::SX::sym("z", MZ_SIZE);
  casadi::SX rho = casadi::SX::sym("rho", 5, NUM_LEG);
  casadi::SX u0 = casadi::SX::sym("u0", MI_SIZE);
  casadi::SX u1 = casadi::SX::sym("u1", MI_SIZE);
  casadi::SX dt = casadi::SX::sym("dt", 1);

  auto x_eig_X = Utils::cas_to_eig(x);
  Eigen::Matrix<casadi::SX, MS_SIZE, 1> x_eig = x_eig_X.head(MS_SIZE);
  auto u0_eig_X = Utils::cas_to_eig(u0);
  Eigen::Matrix<casadi::SX, MI_SIZE, 1> u0_eig = u0_eig_X.head(MI_SIZE);
  auto u1_eig_X = Utils::cas_to_eig(u1);
  Eigen::Matrix<casadi::SX, MI_SIZE, 1> u1_eig = u1_eig_X.head(MI_SIZE);
  auto z_eig_X = Utils::cas_to_eig(z);
  Eigen::Matrix<casadi::SX, MZ_SIZE, 1> z_eig = z_eig_X.head(MZ_SIZE);
  Eigen::Matrix<casadi::SX, 5, NUM_LEG> rho_eig = Utils::cas_to_eigmat<5, NUM_LEG>(rho);

  // discrete time dynamics
  Eigen::Matrix<casadi::SX, MS_SIZE, 1> x_next_eig = mipo_xk1_casadi(x_eig, u0_eig, u1_eig, dt);
  casadi::SX x_next = Utils::eig_to_cas(x_next_eig);
  casadi::SX F = jacobian(x_next, x);
  casadi::SX B = jacobian(x_next, u0);

  mipo_process_dyn_func_ = casadi::Function("process_dyn_func", {x, u0, u1, dt}, {x_next});
  mipo_process_dyn_jac_x_func_ = casadi::Function("process_dyn_jac_x_func", {x, u0, u1, dt}, {F});
  mipo_process_dyn_jac_u_func_ = casadi::Function("process_dyn_jac_u_func", {x, u0, u1, dt}, {B});

  // measurement
  Eigen::Matrix<casadi::SX, MY_SIZE, 1> y_eig = mipo_measurement_casadi<casadi::SX>(x_eig, z_eig, rho_eig);

  casadi::SX y = Utils::eig_to_cas(y_eig);
  // get jacoban of measurement function wrt state
  casadi::SX H = jacobian(y, x);

  mipo_measurement_func_ = casadi::Function("measurement_func", {x, z, rho}, {y});
  mipo_measurement_jac_func_ = casadi::Function("measurement_jac_func", {x, z, rho}, {H});

  return;
}

void MIPOEstimator::mipo_init_noise() {
  // process state noise
  Q1.diagonal().setZero();
  Q1.diagonal().segment<3>(0) = param.proc_n_pos * Eigen::Vector3d::Ones();
  Q1.diagonal().segment<2>(3) = param.proc_n_vel_xy * Eigen::Vector2d::Ones();
  Q1.diagonal()(5) = param.proc_n_vel_z;
  Q1.diagonal().segment<3>(6) = param.proc_n_ang * Eigen::Vector3d::Ones();
  for (int i = 0; i < NUM_LEG; i++) {
    Q1.diagonal().segment<3>(9 + 6 * i) = param.proc_n_foot_pos * Eigen::Vector3d::Ones();
    Q1.diagonal().segment<3>(9 + 6 * i + 3) = param.proc_n_foot_vel * Eigen::Vector3d::Ones();
  }
  Q1.diagonal().segment<3>(9 + 6 * NUM_LEG) = param.proc_n_ba * Eigen::Vector3d::Ones();
  Q1.diagonal().segment<3>(9 + 6 * NUM_LEG + 3) = param.proc_n_bg * Eigen::Vector3d::Ones();
  for (int i = 0; i < NUM_LEG; i++) {
    Q1.diagonal().segment<3>(9 + 6 * NUM_LEG + 6 + 3 * i) = param.proc_n_foot1_ba * Eigen::Vector3d::Ones();
  }

  // process input noise
  Q2.diagonal().setZero();
  Q2.diagonal().segment<3>(0) = param.ctrl_n_acc * Eigen::Vector3d::Ones();
  Q2.diagonal().segment<3>(3) = param.ctrl_n_gyro * Eigen::Vector3d::Ones();
  for (int i = 0; i < NUM_LEG; i++) {
    Q2.diagonal().segment<3>(6 + 3 * i) = param.ctrl_n_foot1_acc * Eigen::Vector3d::Ones();
  }

  // measurement noise
  R.diagonal().setOnes();
  R.diagonal() *= 1e-2;
  for (int i = 0; i < NUM_LEG; i++) {
    R.diagonal().segment<3>(6 + MY_PER_LEG * i) = param.meas_n_rolling_vel * Eigen::Vector3d::Ones();

    R.diagonal()(9 + MY_PER_LEG * i) = param.meas_n_foot_height;
  }
  return;
}

// end
