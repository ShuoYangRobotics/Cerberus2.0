#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <casadi/casadi.hpp>
#include <memory>
#include <string>
#include <vector>

#include "utils/POParams.hpp"

/*
 * SIPO contains a differentiable EKF to estimate the robot state
 * The state is defined as
 *   the process dynamics of attitude-lo-foot filter
    % state x
    %   - position    (1:3)
    %   - velocity    (4:6)
    %   - euler angle (7:9)
    %   - foot1       (10:12)
    %   - foot2       (13:15)
    %   - foot3       (16:18)
    %   - foot4       (19:21)
    %   - acc bias    (22:24)
    %   - gyro bias   (25:27)
    %   - time  tk    (28)

    % control u
    %   - w      (1:3)      IMU angular veolocity
    %   - a       (4:6)     IMU acceleration
    %   - hk       (7)


    % dot x
    %   - velocity
    %   - acc
    %   - deuler
    %   - d foot
    %   - d acc bias
    %   - d gyro bias
    %   - hk
 */

#define SS_SIZE 28
#define SI_SIZE 7
#define SY_SIZE 29
#define SY_PER_LEG 7
#define SZ_SIZE 28

#ifndef NUM_LEG
#define NUM_LEG 4
#endif
#ifndef NUM_DOF
#define NUM_DOF 12
#endif

struct SIPOEstimatorSensorData {
  SIPOEstimatorSensorData() {
    body_acc.setZero();
    body_gyro.setZero();
    joint_angles.setZero();
    joint_velocities.setZero();
    foot_contact.setZero();
  }

  void loadFromVec(Eigen::Matrix<double, 35, 1> sensor_vec);

  // input sensor data
  Eigen::Matrix<double, 3, 1> body_acc;
  Eigen::Matrix<double, 3, 1> body_gyro;
  Eigen::Matrix<double, NUM_DOF, 1> joint_angles;
  Eigen::Matrix<double, NUM_DOF, 1> joint_velocities;
  Eigen::Matrix<double, NUM_LEG, 1> foot_contact;
  double body_yaw = 0.0;
};

class SIPOEstimator {
 public:
  SIPOEstimator();
  ~SIPOEstimator();
  void ekfUpdate(const Eigen::Matrix<double, SS_SIZE, 1>& x_k, const Eigen::Matrix<double, SS_SIZE, SS_SIZE>& P_k,
                 const SIPOEstimatorSensorData& sensor_data_k, const SIPOEstimatorSensorData& sensor_data_k1, const double dt,
                 // output
                 Eigen::Matrix<double, SS_SIZE, 1>& x_k1, Eigen::Matrix<double, SS_SIZE, SS_SIZE>& P_k1);

  // given a sensor data k, initialize the state
  Eigen::Matrix<double, SS_SIZE, 1> ekfInitState(const SIPOEstimatorSensorData& sensor_data_k);
  /*
   * following interface functions are called by the external user
   */
  // input and output are Eigen Vector or Matrix

  Eigen::Matrix<double, SS_SIZE, 1> proc_func(const Eigen::Matrix<double, SS_SIZE, 1> x, const Eigen::Matrix<double, SI_SIZE, 1> u,
                                              double dt);

  Eigen::Matrix<double, SS_SIZE, 1> proc_func(const Eigen::Matrix<double, SS_SIZE, 1> x, const Eigen::Matrix<double, SI_SIZE, 1> u0,
                                              const Eigen::Matrix<double, SI_SIZE, 1> u1, double dt);

  Eigen::Matrix<double, SS_SIZE, SS_SIZE> proc_func_x_jac(const Eigen::Matrix<double, SS_SIZE, 1> x,
                                                          const Eigen::Matrix<double, SI_SIZE, 1> u, double dt);
  Eigen::Matrix<double, SS_SIZE, SS_SIZE> proc_func_x_jac(const Eigen::Matrix<double, SS_SIZE, 1> x,
                                                          const Eigen::Matrix<double, SI_SIZE, 1> u0,
                                                          const Eigen::Matrix<double, SI_SIZE, 1> u1, double dt);

  Eigen::Matrix<double, SS_SIZE, SI_SIZE> proc_func_u_jac(const Eigen::Matrix<double, SS_SIZE, 1> x,
                                                          const Eigen::Matrix<double, SI_SIZE, 1> u, double dt);
  Eigen::Matrix<double, SS_SIZE, SI_SIZE> proc_func_u_jac(const Eigen::Matrix<double, SS_SIZE, 1> x,
                                                          const Eigen::Matrix<double, SI_SIZE, 1> u0,
                                                          const Eigen::Matrix<double, SI_SIZE, 1> u1, double dt);

  // interface functions to evaluate measurement functions numerically
  Eigen::Matrix<double, SY_SIZE, 1> meas_func(const Eigen::Matrix<double, SS_SIZE, 1> x, const Eigen::Matrix<double, SZ_SIZE, 1> z);

  Eigen::Matrix<double, SY_SIZE, SS_SIZE> meas_func_jac(const Eigen::Matrix<double, SS_SIZE, 1> x,
                                                        const Eigen::Matrix<double, SZ_SIZE, 1> z);

 private:
  bool is_initialized_;

  bool sipo_use_foot_ang_contact_model_;

  Eigen::Matrix<double, 5, NUM_LEG> rho_true_;

  // internal symbolic variables and functions, save discrete dynamics,
  /* following functions should not be called from external
   * we currently expose them for debugging purpose
   */
  // process dynamics - eigen/casadi
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, SS_SIZE, 1> sipo_process_dyn_casadi(const Eigen::Matrix<SCALAR_T, SS_SIZE, 1> x,
                                                              const Eigen::Matrix<SCALAR_T, SI_SIZE, 1> u);

  // discrete process dynamics - eigen/casadi
  Eigen::Matrix<double, SS_SIZE, 1> sipo_xk1_eigen(const Eigen::Matrix<double, SS_SIZE, 1> x, const Eigen::Matrix<double, SI_SIZE, 1> u0,
                                                   const Eigen::Matrix<double, SI_SIZE, 1> u1, double dt);

  Eigen::Matrix<casadi::SX, SS_SIZE, 1> sipo_xk1_casadi(const Eigen::Matrix<casadi::SX, SS_SIZE, 1> x,
                                                        const Eigen::Matrix<casadi::SX, SI_SIZE, 1> u0,
                                                        const Eigen::Matrix<casadi::SX, SI_SIZE, 1> u1, casadi::SX dt);

  // measurement functions - eigen/casadi
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, SY_SIZE, 1> sipo_measurement_casadi(const Eigen::Matrix<SCALAR_T, SS_SIZE, 1> x,
                                                              const Eigen::Matrix<SCALAR_T, SZ_SIZE, 1> z,
                                                              const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, NUM_LEG> param);

  // measurement function and their jacobians as casadi::Function(s)
  // create four casadi functions for 1. process dynamics 2. measurement 3.
  // process dynamics jacobian wrt state, 4. process dynamics jacobian wrt
  // input 5. measurement jacobian
  casadi::Function sipo_process_dyn_func_;
  casadi::Function sipo_measurement_func_;
  casadi::Function sipo_process_dyn_jac_x_func_;
  casadi::Function sipo_process_dyn_jac_u_func_;
  casadi::Function sipo_measurement_jac_func_;

  // initalize casadi symbolic variables and functions
  void sipo_init_casadi();

  // initialize noise covariance matrices
  Eigen::DiagonalMatrix<double, SS_SIZE> Q1;
  Eigen::DiagonalMatrix<double, SI_SIZE> Q2;
  Eigen::DiagonalMatrix<double, SY_SIZE> R;
  const POParams param;
  void sipo_init_noise();
};