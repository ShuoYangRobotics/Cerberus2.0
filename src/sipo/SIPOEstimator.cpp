#include <Eigen/SparseQR>

#include "sipo/SIPOEstimator.hpp"
#include "utils/casadi_kino.hpp"
#include "utils/utils.hpp"

void SIPOEstimatorSensorData::loadFromVec(Eigen::Matrix<double, 35, 1> sensor_vec) {
  body_gyro = sensor_vec.segment<3>(0);
  body_acc = sensor_vec.segment<3>(3);
  joint_angles = sensor_vec.segment<12>(6);
  joint_velocities = sensor_vec.segment<12>(18);
  foot_contact = sensor_vec.segment<4>(30);
  body_yaw = sensor_vec(34);
}

SIPOEstimator::SIPOEstimator() {
  rho_true_ = Eigen::Matrix<double, 5, NUM_LEG>::Zero();
  rho_true_ << 0.1805, 0.1805, -0.1805, -0.1805, 0.047, -0.047, 0.047, -0.047, 0.0838, -0.0838, 0.0838, -0.0838, 0.2, 0.2, 0.2, 0.2, 0.2,
      0.2, 0.2, 0.2;

  // must be called every time at the beginning
  sipo_init_casadi();
  sipo_init_noise();
}

SIPOEstimator::~SIPOEstimator() {}

void SIPOEstimator::ekfUpdate(const Eigen::Matrix<double, SS_SIZE, 1>& x_k, const Eigen::Matrix<double, SS_SIZE, SS_SIZE>& P_k,
                              const SIPOEstimatorSensorData& sensor_data_k, const SIPOEstimatorSensorData& sensor_data_k1, const double dt,
                              // output
                              Eigen::Matrix<double, SS_SIZE, 1>& x_k1, Eigen::Matrix<double, SS_SIZE, SS_SIZE>& P_k1,
                              Eigen::Matrix<double, NUM_LEG, 1>& contact_est) {
  Eigen::Matrix<double, SI_SIZE, 1> u_k;
  u_k.segment<3>(0) = sensor_data_k.body_gyro;  // notice the order, defined in sipo_process_dyn_casadi
  u_k.segment<3>(3) = sensor_data_k.body_acc;
  u_k(SI_SIZE - 1) = dt;
  Eigen::Matrix<double, SI_SIZE, 1> u_k1;
  u_k1.segment<3>(0) = sensor_data_k1.body_gyro;  // notice the order, defined in sipo_process_dyn_casadi
  u_k1.segment<3>(3) = sensor_data_k1.body_acc;
  u_k1(SI_SIZE - 1) = dt;

  //   // assemble measurement vector z
  Eigen::Matrix<double, SZ_SIZE, 1> z_k;
  z_k.segment<3>(0) = sensor_data_k.body_gyro;
  z_k.segment<NUM_DOF>(3) = sensor_data_k.joint_angles;
  z_k.segment<NUM_DOF>(3 + NUM_DOF) = sensor_data_k.joint_velocities;
  z_k(3 + 2 * NUM_DOF) = sensor_data_k.body_yaw;

  // get foot force and process to contact flag
  Eigen::Vector4d foot_force = sensor_data_k.foot_contact;
  double threshold_value = 100.0;
  Eigen::Vector4i result = foot_force.unaryExpr([threshold_value](double x) { return x > threshold_value ? 1 : 0; });
  Eigen::Vector4d ck = result.cast<double>();
  contact_est = ck;
  // EKF steps
  // 1. predict

  Eigen::Matrix<double, SS_SIZE, 1> x_k1_pred = proc_func(x_k, u_k, u_k1, dt);
  //   std::cout << "x_k1_pred: " << x_k1_pred.transpose() << std::endl;
  Eigen::Matrix<double, SS_SIZE, SS_SIZE> F = proc_func_x_jac(x_k, u_k, u_k1, dt);
  Eigen::Matrix<double, SS_SIZE, SI_SIZE> G = proc_func_u_jac(x_k, u_k, u_k1, dt);
  // Eigen::SparseMatrix<double> sparse_P_k = P_k.sparseView();
  Eigen::Matrix<double, SS_SIZE, SS_SIZE> P_k_pred = F * P_k * F.transpose();
  // change Q according to contact
  for (int i = 0; i < NUM_LEG; i++) {
    Q1.diagonal().segment<3>(9 + 3 * i) = (1 + (1 - ck(i)) * 1e5) * param.proc_n_foot_pos * Eigen::Vector3d::Ones();

    R.diagonal().segment<3>(SY_PER_LEG * i) = (1 + (1 - ck(i)) * 1e5) * param.meas_n_fk_pos * dt * Eigen::Vector3d::Ones();
    R.diagonal().segment<3>(SY_PER_LEG * i + 3) = (1 + (1 - ck(i)) * 1e5) * param.meas_n_fk_vel * dt * Eigen::Vector3d::Ones();
    R.diagonal()(SY_PER_LEG * i + 6) = (1 + (1 - ck(i)) * 1e5) * param.meas_n_foot_height * dt;
  }
  for (int i = 0; i < SS_SIZE; i++) {
    P_k_pred(i, i) += dt * Q1.diagonal()(i);
  }  // do this to prevent a strange bug mum map invalid pointer

  Eigen::Matrix<double, SS_SIZE, SS_SIZE> GQ2Gt = G * Q2 * G.transpose() * dt;
  P_k_pred += GQ2Gt;

  // 2. update
  Eigen::Matrix<double, SY_SIZE, 1> y_k = meas_func(x_k1_pred, z_k);
  //   std::cout << "y_k: " << y_k.transpose() << std::endl;
  Eigen::Matrix<double, SY_SIZE, SS_SIZE> H = meas_func_jac(x_k1_pred, z_k);
  //   std::cout << "H: " << H << std::endl;
  Eigen::Matrix<double, SY_SIZE, SY_SIZE> S = H * P_k_pred * H.transpose();
  for (int i = 0; i < SY_SIZE; i++) {
    S(i, i) += R.diagonal()(i);
  }

  Eigen::Matrix<int, SY_SIZE, 1> outlier_mask;
  outlier_mask.setOnes();
  Eigen::SparseMatrix<double> mask_sparse(outlier_mask.size(), outlier_mask.sum());
  int col_idx = 0;
  for (int i = 0; i < outlier_mask.size(); i++) {
    if (outlier_mask(i) == 1) {
      mask_sparse.insert(i, col_idx) = 1;
      col_idx++;
    }
  }
  Eigen::VectorXd y_k_masked = mask_sparse.transpose() * y_k;
  Eigen::MatrixXd H_masked = mask_sparse.transpose() * H;
  Eigen::MatrixXd S_masked = mask_sparse.transpose() * S * mask_sparse;

  // 2.2 update, using colhouseholderqr to solve the linear system
  Eigen::VectorXd Sy = S_masked.ldlt().solve(y_k_masked);

  Eigen::MatrixXd tmp = P_k_pred * H_masked.transpose();
  Eigen::VectorXd update = tmp * Sy;
  //   std::cout << "update: " << update.transpose() << std::endl;
  x_k1 = x_k1_pred - update;

  P_k1 = P_k_pred - tmp * S_masked.ldlt().solve(H_masked) * P_k_pred;

  // regularize P_k1 to make it symmetric
  P_k1 = (P_k1 + P_k1.transpose()) / 2.0;

  return;
}

Eigen::Matrix<double, SS_SIZE, 1> SIPOEstimator::ekfInitState(const SIPOEstimatorSensorData& sensor_data_k) {
  Eigen::Matrix<double, SS_SIZE, 1> x_k;
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

    x_k.segment<3>(9 + i * 3) = R_er * foot_pos + init_pos;
  }
  return x_k;
}

/*
 * following interface functions are called by the external user
 */
// input and output are Eigen Vector or sparse Matrix

Eigen::Matrix<double, SS_SIZE, 1> SIPOEstimator::proc_func(const Eigen::Matrix<double, SS_SIZE, 1> x,
                                                           const Eigen::Matrix<double, SI_SIZE, 1> u, double dt) {
  return sipo_xk1_eigen(x, u, u, dt);
}

Eigen::Matrix<double, SS_SIZE, 1> SIPOEstimator::proc_func(const Eigen::Matrix<double, SS_SIZE, 1> x,
                                                           const Eigen::Matrix<double, SI_SIZE, 1> u0,
                                                           const Eigen::Matrix<double, SI_SIZE, 1> u1, double dt) {
  return sipo_xk1_eigen(x, u0, u1, dt);
}

Eigen::Matrix<double, SS_SIZE, SS_SIZE> SIPOEstimator::proc_func_x_jac(const Eigen::Matrix<double, SS_SIZE, 1> x,
                                                                       const Eigen::Matrix<double, SI_SIZE, 1> u, double dt) {
  // convert x and u to casadi DM type
  casadi::DM x_dm = Utils::eig_to_cas_DM<SS_SIZE, 1>(x);
  casadi::DM u_dm = Utils::eig_to_cas_DM<SI_SIZE, 1>(u);
  // covert rho to casadi DM type
  casadi::DM dt_dm = casadi::DM(dt);
  std::vector<casadi::DM> arg = {x_dm, u_dm, u_dm, dt_dm};
  std::vector<casadi::DM> res = sipo_process_dyn_jac_x_func_(arg);
  // convert the result to eigen matrix
  Eigen::Matrix<double, SS_SIZE, SS_SIZE> SpMatrx = Utils::cas_DM_to_eig_mat<SS_SIZE, SS_SIZE>(res[0]);

  return SpMatrx;
}

Eigen::Matrix<double, SS_SIZE, SS_SIZE> SIPOEstimator::proc_func_x_jac(const Eigen::Matrix<double, SS_SIZE, 1> x,
                                                                       const Eigen::Matrix<double, SI_SIZE, 1> u0,
                                                                       const Eigen::Matrix<double, SI_SIZE, 1> u1, double dt) {
  // convert x and u to casadi DM type
  casadi::DM x_dm = Utils::eig_to_cas_DM<SS_SIZE, 1>(x);
  casadi::DM u0_dm = Utils::eig_to_cas_DM<SI_SIZE, 1>(u0);
  casadi::DM u1_dm = Utils::eig_to_cas_DM<SI_SIZE, 1>(u1);
  // covert rho to casadi DM type
  casadi::DM dt_dm = casadi::DM(dt);
  std::vector<casadi::DM> arg = {x_dm, u0_dm, u1_dm, dt_dm};
  std::vector<casadi::DM> res = sipo_process_dyn_jac_x_func_(arg);
  // convert the result to eigen matrix
  Eigen::Matrix<double, SS_SIZE, SS_SIZE> SpMatrx = Utils::cas_DM_to_eig_mat<SS_SIZE, SS_SIZE>(res[0]);

  return SpMatrx;
}

Eigen::Matrix<double, SS_SIZE, SI_SIZE> SIPOEstimator::proc_func_u_jac(const Eigen::Matrix<double, SS_SIZE, 1> x,
                                                                       const Eigen::Matrix<double, SI_SIZE, 1> u, double dt) {
  // convert x and u to casadi DM type
  casadi::DM x_dm = Utils::eig_to_cas_DM<SS_SIZE, 1>(x);
  casadi::DM u_dm = Utils::eig_to_cas_DM<SI_SIZE, 1>(u);
  // covert rho to casadi DM type
  casadi::DM dt_dm = casadi::DM(dt);
  std::vector<casadi::DM> arg = {x_dm, u_dm, u_dm, dt_dm};
  std::vector<casadi::DM> res = sipo_process_dyn_jac_u_func_(arg);
  // convert the result to eigen matrix
  Eigen::Matrix<double, SS_SIZE, SI_SIZE> SpMatrx = Utils::cas_DM_to_eig_mat<SS_SIZE, SI_SIZE>(res[0]);

  return SpMatrx;
}

Eigen::Matrix<double, SS_SIZE, SI_SIZE> SIPOEstimator::proc_func_u_jac(const Eigen::Matrix<double, SS_SIZE, 1> x,
                                                                       const Eigen::Matrix<double, SI_SIZE, 1> u0,
                                                                       const Eigen::Matrix<double, SI_SIZE, 1> u1, double dt) {
  // convert x and u to casadi DM type
  casadi::DM x_dm = Utils::eig_to_cas_DM<SS_SIZE, 1>(x);
  casadi::DM u0_dm = Utils::eig_to_cas_DM<SI_SIZE, 1>(u0);
  casadi::DM u1_dm = Utils::eig_to_cas_DM<SI_SIZE, 1>(u1);
  // covert rho to casadi DM type
  casadi::DM dt_dm = casadi::DM(dt);
  std::vector<casadi::DM> arg = {x_dm, u0_dm, u1_dm, dt_dm};
  std::vector<casadi::DM> res = sipo_process_dyn_jac_u_func_(arg);
  // convert the result to eigen matrix
  Eigen::Matrix<double, SS_SIZE, SI_SIZE> SpMatrx = Utils::cas_DM_to_eig_mat<SS_SIZE, SI_SIZE>(res[0]);

  return SpMatrx;
}

// measurement function can directly use the template funtion
Eigen::Matrix<double, SY_SIZE, 1> SIPOEstimator::meas_func(const Eigen::Matrix<double, SS_SIZE, 1> x,
                                                           const Eigen::Matrix<double, SZ_SIZE, 1> z) {
  return sipo_measurement_casadi<double>(x, z, rho_true_);
}

// meausrement jacobian must call the casadi function
// but the casadi function can be generated first
Eigen::Matrix<double, SY_SIZE, SS_SIZE> SIPOEstimator::meas_func_jac(const Eigen::Matrix<double, SS_SIZE, 1> x,
                                                                     const Eigen::Matrix<double, SZ_SIZE, 1> z) {
  // convert x and z to casadi DM type
  casadi::DM x_dm = Utils::eig_to_cas_DM<SS_SIZE, 1>(x);
  casadi::DM z_dm = Utils::eig_to_cas_DM<SZ_SIZE, 1>(z);
  // covert rho to casadi DM type
  casadi::DM rho_dm = Utils::eig_to_cas_DM<5, NUM_LEG>(rho_true_);
  std::vector<casadi::DM> arg = {x_dm, z_dm, rho_dm};
  std::vector<casadi::DM> res = sipo_measurement_jac_func_(arg);
  // convert the result to eigen matrix
  Eigen::Matrix<double, SY_SIZE, SS_SIZE> SpMatrx = Utils::cas_DM_to_eig_mat<SY_SIZE, SS_SIZE>(res[0]);

  return SpMatrx;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, SS_SIZE, 1> SIPOEstimator::sipo_process_dyn_casadi(const Eigen::Matrix<SCALAR_T, SS_SIZE, 1> x,
                                                                           const Eigen::Matrix<SCALAR_T, SI_SIZE, 1> u) {
  Eigen::Matrix<SCALAR_T, 3, 1> pos = x.segment(0, 3);
  Eigen::Matrix<SCALAR_T, 3, 1> vel = x.segment(3, 3);
  Eigen::Matrix<SCALAR_T, 3, 1> euler = x.segment(6, 3);
  // segment(9, 3), segment(12, 3), segment(15, 3), segment(18, 3) are foot
  // positions
  Eigen::Matrix<SCALAR_T, 3, 1> ba = x.segment(21, 3);  // body acc bias
  Eigen::Matrix<SCALAR_T, 3, 1> bg = x.segment(24, 3);  // gyro bias

  Eigen::Matrix<SCALAR_T, 3, 1> w = u.segment(0, 3) - bg;  // body angular velocity
  Eigen::Matrix<SCALAR_T, 3, 1> a = u.segment(3, 3) - ba;  // body linear acceleration

  Eigen::Matrix<SCALAR_T, 3, 1> deuler = legged::mtx_w_to_euler_dot<SCALAR_T>(euler) * w;

  Eigen::Matrix<SCALAR_T, 3, 3> R = legged::euler_to_rot<SCALAR_T>(euler);

  Eigen::Matrix<SCALAR_T, 3, 1> gravity(0, 0, 9.81);
  Eigen::Matrix<SCALAR_T, 3, 1> acc = R * a - gravity;

  Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> xdot(SS_SIZE);
  xdot << vel, acc, deuler, Eigen::Matrix<SCALAR_T, 18, 1>::Zero(), 1;

  return xdot;
}

Eigen::Matrix<casadi::SX, SS_SIZE, 1> SIPOEstimator::sipo_xk1_casadi(const Eigen::Matrix<casadi::SX, SS_SIZE, 1> x,
                                                                     const Eigen::Matrix<casadi::SX, SI_SIZE, 1> u0,
                                                                     const Eigen::Matrix<casadi::SX, SI_SIZE, 1> u1, casadi::SX dt) {
  // use rk4 function to get xk+1
  // pass SIPOEstimator::sipo_process_dyn_casadi as a std::function pointer
  Eigen::Matrix<casadi::SX, SS_SIZE, 1> x_k1 = legged::dyn_rk4_casadi<casadi::SX, SS_SIZE, SI_SIZE>(
      x, u0, u1, dt, std::bind(&SIPOEstimator::sipo_process_dyn_casadi<casadi::SX>, this, std::placeholders::_1, std::placeholders::_2));

  return x_k1;
}

Eigen::Matrix<double, SS_SIZE, 1> SIPOEstimator::sipo_xk1_eigen(const Eigen::Matrix<double, SS_SIZE, 1> x,
                                                                const Eigen::Matrix<double, SI_SIZE, 1> u0,
                                                                const Eigen::Matrix<double, SI_SIZE, 1> u1, double dt) {
  // use rk4 function to get xk+1
  // pass SIPOEstimator::sipo_process_dyn_casadi as a std::function pointer
  Eigen::Matrix<double, SS_SIZE, 1> x_k1 = legged::dyn_rk4_casadi<double, SS_SIZE, SI_SIZE>(
      x, u0, u1, dt, std::bind(&SIPOEstimator::sipo_process_dyn_casadi<double>, this, std::placeholders::_1, std::placeholders::_2));

  return x_k1;
}
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, SY_SIZE, 1> SIPOEstimator::sipo_measurement_casadi(const Eigen::Matrix<SCALAR_T, SS_SIZE, 1> x,
                                                                           const Eigen::Matrix<SCALAR_T, SZ_SIZE, 1> z,
                                                                           const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, NUM_LEG> param) {
  Eigen::Matrix<SCALAR_T, 3, 1> wk = z.segment(0, 3);
  Eigen::Matrix<SCALAR_T, 12, 1> phik = z.segment(3, 12);
  Eigen::Matrix<SCALAR_T, 12, 1> dphik = z.segment(12 + 3, 12);
  SCALAR_T yawk = z(12 + 3 + 12);

  // states
  Eigen::Matrix<SCALAR_T, 3, 1> pos = x.segment(0, 3);
  Eigen::Matrix<SCALAR_T, 3, 1> vel = x.segment(3, 3);
  Eigen::Matrix<SCALAR_T, 3, 1> euler = x.segment(6, 3);
  Eigen::Matrix<SCALAR_T, 3, 3> R_er = legged::euler_to_rot<SCALAR_T>(euler);
  Eigen::Matrix<SCALAR_T, 12, 1> foot_pos;
  foot_pos << x.segment(9, 3), x.segment(12, 3), x.segment(15, 3), x.segment(18, 3);

  Eigen::Matrix<SCALAR_T, 3, 1> bg = x.segment(24, 3);

  Eigen::Matrix<SCALAR_T, SY_SIZE, 1> meas_residual;
  meas_residual.setZero();
  for (int i = 0; i < NUM_LEG; ++i) {
    Eigen::Matrix<SCALAR_T, 3, 1> angle = phik.segment(i * 3, 3);
    Eigen::Matrix<SCALAR_T, 3, 1> av = dphik.segment(i * 3, 3);
    Eigen::Matrix<SCALAR_T, -1, 1> param_leg = param.col(i);
    Eigen::Matrix<SCALAR_T, 3, 1> p_rf = legged::fk_pf_pos(angle, param_leg);
    Eigen::Matrix<SCALAR_T, 3, 3> J_rf = legged::d_fk_dt(angle, param_leg);
    Eigen::Matrix<SCALAR_T, 3, 1> w_k_no_bias = wk - bg;
    Eigen::Matrix<SCALAR_T, 3, 1> leg_v = (J_rf * av + legged::skewSymmetric<SCALAR_T>(w_k_no_bias) * p_rf);

    meas_residual.segment(i * SY_PER_LEG, 3) = p_rf - R_er.transpose() * (foot_pos.segment(i * 3, 3) - pos);
    meas_residual.segment(i * SY_PER_LEG + 3, 3) = (vel + R_er * leg_v);

    meas_residual(i * SY_PER_LEG + 6) = foot_pos(i * 3 + 2);
  }

  meas_residual(SY_SIZE - 1) = yawk - euler(2);

  return meas_residual;
}
void SIPOEstimator::sipo_init_casadi() {
  // define casadi symbols for state, input, parameters, sensor data, and
  // parameters
  casadi::SX x = casadi::SX::sym("x", SS_SIZE);
  casadi::SX z = casadi::SX::sym("z", SZ_SIZE);
  casadi::SX rho = casadi::SX::sym("rho", 5, NUM_LEG);
  casadi::SX u0 = casadi::SX::sym("u0", SI_SIZE);
  casadi::SX u1 = casadi::SX::sym("u1", SI_SIZE);
  casadi::SX dt = casadi::SX::sym("dt", 1);

  auto x_eig_X = Utils::cas_to_eig(x);
  Eigen::Matrix<casadi::SX, SS_SIZE, 1> x_eig = x_eig_X.head(SS_SIZE);
  auto u0_eig_X = Utils::cas_to_eig(u0);
  Eigen::Matrix<casadi::SX, SI_SIZE, 1> u0_eig = u0_eig_X.head(SI_SIZE);
  auto u1_eig_X = Utils::cas_to_eig(u1);
  Eigen::Matrix<casadi::SX, SI_SIZE, 1> u1_eig = u1_eig_X.head(SI_SIZE);
  auto z_eig_X = Utils::cas_to_eig(z);
  Eigen::Matrix<casadi::SX, SZ_SIZE, 1> z_eig = z_eig_X.head(SZ_SIZE);
  Eigen::Matrix<casadi::SX, 5, NUM_LEG> rho_eig = Utils::cas_to_eigmat<5, NUM_LEG>(rho);

  // discrete time dynamics
  Eigen::Matrix<casadi::SX, SS_SIZE, 1> x_next_eig = sipo_xk1_casadi(x_eig, u0_eig, u1_eig, dt);
  casadi::SX x_next = Utils::eig_to_cas(x_next_eig);
  casadi::SX F = jacobian(x_next, x);
  casadi::SX B = jacobian(x_next, u0);

  sipo_process_dyn_func_ = casadi::Function("process_dyn_func", {x, u0, u1, dt}, {x_next});
  sipo_process_dyn_jac_x_func_ = casadi::Function("process_dyn_jac_x_func", {x, u0, u1, dt}, {F});
  sipo_process_dyn_jac_u_func_ = casadi::Function("process_dyn_jac_u_func", {x, u0, u1, dt}, {B});

  // measurement
  Eigen::Matrix<casadi::SX, SY_SIZE, 1> y_eig = sipo_measurement_casadi<casadi::SX>(x_eig, z_eig, rho_eig);

  casadi::SX y = Utils::eig_to_cas(y_eig);
  // get jacoban of measurement function wrt state
  casadi::SX H = jacobian(y, x);

  sipo_measurement_func_ = casadi::Function("measurement_func", {x, z, rho}, {y});
  sipo_measurement_jac_func_ = casadi::Function("measurement_jac_func", {x, z, rho}, {H});

  return;
}

void SIPOEstimator::sipo_init_noise() {
  // process state noise
  Q1.diagonal().setZero();
  Q1.diagonal().segment<3>(0) = param.proc_n_pos * Eigen::Vector3d::Ones();
  Q1.diagonal().segment<2>(3) = param.proc_n_vel_xy * Eigen::Vector2d::Ones();
  Q1.diagonal()(5) = param.proc_n_vel_z;
  Q1.diagonal().segment<3>(6) = param.proc_n_ang * Eigen::Vector3d::Ones();
  for (int i = 0; i < NUM_LEG; i++) {
    Q1.diagonal().segment<3>(9 + 3 * i) = param.proc_n_foot_pos * Eigen::Vector3d::Ones();
  }
  Q1.diagonal().segment<3>(9 + 3 * NUM_LEG) = param.proc_n_ba * Eigen::Vector3d::Ones();
  Q1.diagonal().segment<3>(9 + 3 * NUM_LEG + 3) = param.proc_n_bg * Eigen::Vector3d::Ones();

  // process input noise
  Q2.diagonal().setZero();
  Q2.diagonal().segment<3>(0) = param.ctrl_n_acc * Eigen::Vector3d::Ones();
  Q2.diagonal().segment<3>(3) = param.ctrl_n_gyro * Eigen::Vector3d::Ones();

  // measurement noise
  R.diagonal().setOnes();
  R.diagonal() *= 1e-2;

  return;
}