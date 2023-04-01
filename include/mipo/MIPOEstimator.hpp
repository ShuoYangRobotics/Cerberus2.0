#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <casadi/casadi.hpp>
#include <memory>
#include <string>
#include <vector>

/*
 * MIPO contains a differentiable EKF to estimate the robot state
 * The state is defined as
 *   the process dynamics of attitude-lo-foot filter
     but with additional IMUs on feet
     state x
       - position          (1:3)
       - velocity          (4:6)
       - euler angle       (7:9)
       - foot1 pos         (10:12)
       - foot1 vel         (13:15)
       - foot2 pos         (16:18)
       - foot2 vel         (19:21)
       - foot3 pos         (22:24)
       - foot3 vel         (25:27)
       - foot4 pos         (28:30)
       - foot4 vel         (31:33)
       - body acc bias     (34:36)
       - body gyro bias    (37:39)
       - foot1 acc bias    (40:42)
       - foot2 acc bias    (43:45)
       - foot3 acc bias    (46:48)
       - foot4 acc bias    (49:57)
       - time  tk          (52)     //not used

     input u
       - w      (1:3)      body IMU angular veolocity
       - a       (4:6)     body IMU acceleration
       - a1      (7:9)     foot 1 IMU acceleration (already in body frame)
       - a2      (10:12)   foot 2 IMU acceleration (already in body frame)
       - a3      (13:15)   foot 3 IMU acceleration (already in body frame)
       - a4      (16:18)   foot 4 IMU acceleration (already in body frame)
       - hk       (19)             //not used

     dot x
       - velocity
       - acc
       - deuler
       - foot1 vel
       - foot1 acc
       - foot2 vel
       - foot2 acc
       - foot3 vel
       - foot3 acc
       - foot4 vel
       - foot4 acc
       - hk
 */
#define MS_SIZE 52    // state size
#define MI_SIZE 19    // input size
#define MY_SIZE 45    // measurement size NUM_LEG*MY_PER_LEG+1
#define MZ_SIZE 40    // sensor data size
#define NUM_LEG 4     // NUM of legs
#define NUM_DOF 12    // NUM of leg motors
#define MY_PER_LEG 11 // measurement size per leg

// a sensor data structure, all vectors are fixed size Eigen Vectors
//[body IMU gyro (3), body IMU acc (3), joint angles
//  (12), joint angle velocities (12), foot1 IMU acc (3), foot2 IMU acc (3),
//  foot3 IMU acc (3), foot4 IMU acc (3), foot1 IMU gyro (3), foot2 IMU gyro
//  (3), foot3 IMU gyro (3), foot4 IMU gyro (3) yaw]
struct MIPOEstimatorSensorData {
  MIPOEstimatorSensorData() {
    body_acc.setZero();
    body_gyro.setZero();
    joint_angles.setZero();
    joint_velocities.setZero();
    foot_acc.setZero();
    foot_gyro.setZero();

    // init R_fi_list to be
    // foot IMU frame has a transformation wrt foot center
    // look at slide 5 of
    // https://docs.google.com/presentation/d/1rUN62W7BCPNz4ljc4ZMbSyKkZgsfU4hKySbO72XpJJ4/edit#slide=id.g1428358afc4_1_0
    // FL FR RL RR
    // {[-1  0  0;
    // 0 0 - 1;
    //            0 -1  0],
    //          [-1   0   0;
    //            0   0   1;
    //            0   1   0],
    //          [-1  0  0;
    //            0  0 -1;
    //            0 -1  0],
    //          [-1   0  0;
    //            0   0  1;
    //            0   1  0]
    Eigen::Matrix3d R_fi_fl;
    R_fi_fl << -1, 0, 0, 0, 0, -1, 0, -1, 0;
    R_fi_list.push_back(R_fi_fl);
    Eigen::Matrix3d R_fi_fr;
    R_fi_fr << -1, 0, 0, 0, 0, 1, 0, 1, 0;
    R_fi_list.push_back(R_fi_fr);
    Eigen::Matrix3d R_fi_rl;
    R_fi_rl << -1, 0, 0, 0, 0, -1, 0, -1, 0;
    R_fi_list.push_back(R_fi_rl);
    Eigen::Matrix3d R_fi_rr;
    R_fi_rr << -1, 0, 0, 0, 0, 1, 0, 1, 0;
    R_fi_list.push_back(R_fi_rr);
  }

  // convert foot IMU data to robot body frame
  void footIMUToBody();
  void loadFromVec(Eigen::Matrix<double, 55, 1> sensor_vec);

  // input sensor data
  Eigen::Matrix<double, 3, 1> body_acc;
  Eigen::Matrix<double, 3, 1> body_gyro;
  Eigen::Matrix<double, NUM_DOF, 1> joint_angles;
  Eigen::Matrix<double, NUM_DOF, 1> joint_velocities;
  Eigen::Matrix<double, 3, NUM_LEG> foot_acc;
  Eigen::Matrix<double, 3, NUM_LEG> foot_gyro;
  double body_yaw = 0.0;

  // processed
  Eigen::Matrix<double, 3, NUM_LEG> foot_acc_body;
  Eigen::Matrix<double, 3, NUM_LEG> foot_gyro_body;

  // store transformations from foot IMU internal frame to foot center frame
  std::vector<Eigen::Matrix3d> R_fi_list;
};

struct MIPOParams {
  double init_cov = 0.1;
  double init_bias_cov = 1e-4;
  double init_body_height = 0.3;

  int data_start_idx = 200;

  double proc_n_pos = 0.0005;
  double proc_n_vel_xy = 0.005;
  double proc_n_vel_z = 0.005;
  double proc_n_ang = 1e-7;
  double proc_n_foot_pos = 1e-4;
  double proc_n_foot_vel = 2;
  double proc_n_ba = 1e-4;
  double proc_n_bg = 1e-5;
  double proc_n_foot1_ba = 1e-4;
  double proc_n_foot2_ba = 1e-4;
  double proc_n_foot3_ba = 1e-4;
  double proc_n_foot4_ba = 1e-4;

  double ctrl_n_acc = 1e-1;
  double ctrl_n_gyro = 1e-3;
  double ctrl_n_foot1_acc = 1e-1;
  double ctrl_n_foot2_acc = 1e-1;
  double ctrl_n_foot3_acc = 1e-1;
  double ctrl_n_foot4_acc = 1e-1;

  double meas_n_fk_pos = 0.001;
  double meas_n_fk_vel = 0.01;
  double meas_n_foot_height = 0.001;
  double meas_n_rolling_vel = 0.01;
};

class MIPOEstimator {
public:
  MIPOEstimator();
  ~MIPOEstimator();

  /* important EKF function,
     * input state x_k (Eigen Vector), covariance P_k (Eigen sparse matrix),
     sensor data
     * split sensor data to input and measurement
     * predict  state and covariance then measurement
     * update state and covariance output state x_k+1 (Eigen Vector), covariance
     P_k+1 (Eigen sparse matrix)

     * sensor data format: [body IMU gyro (3), body IMU acc (3), joint angles
     (12), joint angle velocities (12), foot1 IMU acc (3), foot2 IMU acc (3),
     foot3 IMU acc (3), foot4 IMU acc (3), foot1 IMU gyro (3), foot2 IMU gyro
     (3), foot3 IMU gyro (3), foot4 IMU gyro (3) yaw], indices are [0:2, 3:5,
     6:17, 18:29, 30:32, 33:35, 36:38, 39:41, 42:44, 45:47, 48:50, 51:53 54]

   */
  void ekfUpdate(const Eigen::Matrix<double, MS_SIZE, 1> &x_k,
                 const Eigen::Matrix<double, MS_SIZE, MS_SIZE> &P_k,
                 const MIPOEstimatorSensorData &sensor_data_k,
                 const MIPOEstimatorSensorData &sensor_data_k1, const double dt,
                 // output
                 Eigen::Matrix<double, MS_SIZE, 1> &x_k1,
                 Eigen::Matrix<double, MS_SIZE, MS_SIZE> &P_k1);

  // given a sensor data k, initialize the state
  Eigen::Matrix<double, MS_SIZE, 1>
  ekfInitState(const MIPOEstimatorSensorData &sensor_data_k);

  /*
   * following interface functions are called by the external user
   */
  // input and output are Eigen Vector or Matrix

  Eigen::Matrix<double, MS_SIZE, 1>
  proc_func(const Eigen::Matrix<double, MS_SIZE, 1> x,
            const Eigen::Matrix<double, MI_SIZE, 1> u, double dt);

  Eigen::Matrix<double, MS_SIZE, 1>
  proc_func(const Eigen::Matrix<double, MS_SIZE, 1> x,
            const Eigen::Matrix<double, MI_SIZE, 1> u0,
            const Eigen::Matrix<double, MI_SIZE, 1> u1, double dt);

  Eigen::Matrix<double, MS_SIZE, MS_SIZE>
  proc_func_x_jac(const Eigen::Matrix<double, MS_SIZE, 1> x,
                  const Eigen::Matrix<double, MI_SIZE, 1> u, double dt);
  Eigen::Matrix<double, MS_SIZE, MS_SIZE>
  proc_func_x_jac(const Eigen::Matrix<double, MS_SIZE, 1> x,
                  const Eigen::Matrix<double, MI_SIZE, 1> u0,
                  const Eigen::Matrix<double, MI_SIZE, 1> u1, double dt);

  Eigen::Matrix<double, MS_SIZE, MI_SIZE>
  proc_func_u_jac(const Eigen::Matrix<double, MS_SIZE, 1> x,
                  const Eigen::Matrix<double, MI_SIZE, 1> u, double dt);
  Eigen::Matrix<double, MS_SIZE, MI_SIZE>
  proc_func_u_jac(const Eigen::Matrix<double, MS_SIZE, 1> x,
                  const Eigen::Matrix<double, MI_SIZE, 1> u0,
                  const Eigen::Matrix<double, MI_SIZE, 1> u1, double dt);

  // interface functions to evaluate measurement functions numerically
  Eigen::Matrix<double, MY_SIZE, 1>
  meas_func(const Eigen::Matrix<double, MS_SIZE, 1> x,
            const Eigen::Matrix<double, MZ_SIZE, 1> z);

  Eigen::Matrix<double, MY_SIZE, MS_SIZE>
  meas_func_jac(const Eigen::Matrix<double, MS_SIZE, 1> x,
                const Eigen::Matrix<double, MZ_SIZE, 1> z);

private:
  bool is_initialized_;

  bool mipo_use_foot_ang_contact_model_;

  Eigen::Matrix<double, 5, NUM_LEG> rho_true_;

  // internal symbolic variables and functions, save discrete dynamics,
  /* following functions should not be called from external
   * we currently expose them for debugging purpose
   */
  // process dynamics - eigen/casadi
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, MS_SIZE, 1>
  mipo_process_dyn_casadi(const Eigen::Matrix<SCALAR_T, MS_SIZE, 1> x,
                          const Eigen::Matrix<SCALAR_T, MI_SIZE, 1> u);

  // discrete process dynamics - eigen/casadi
  Eigen::Matrix<double, MS_SIZE, 1>
  mipo_xk1_eigen(const Eigen::Matrix<double, MS_SIZE, 1> x,
                 const Eigen::Matrix<double, MI_SIZE, 1> u0,
                 const Eigen::Matrix<double, MI_SIZE, 1> u1, double dt);

  Eigen::Matrix<casadi::SX, MS_SIZE, 1>
  mipo_xk1_casadi(const Eigen::Matrix<casadi::SX, MS_SIZE, 1> x,
                  const Eigen::Matrix<casadi::SX, MI_SIZE, 1> u0,
                  const Eigen::Matrix<casadi::SX, MI_SIZE, 1> u1,
                  casadi::SX dt);

  // measurement functions - eigen/casadi
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, MY_SIZE, 1> mipo_measurement_casadi(
      const Eigen::Matrix<SCALAR_T, MS_SIZE, 1> x,
      const Eigen::Matrix<SCALAR_T, MZ_SIZE, 1> z,
      const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, NUM_LEG> param);

  // measurement function and their jacobians as casadi::Function(s)
  // create four casadi functions for 1. process dynamics 2. measurement 3.
  // process dynamics jacobian wrt state, 4. process dynamics jacobian wrt
  // input 5. measurement jacobian
  casadi::Function mipo_process_dyn_func_;
  casadi::Function mipo_measurement_func_;
  casadi::Function mipo_process_dyn_jac_x_func_;
  casadi::Function mipo_process_dyn_jac_u_func_;
  casadi::Function mipo_measurement_jac_func_;

  // initalize casadi symbolic variables and functions
  void mipo_init_casadi();

  // initialize noise covariance matrices
  Eigen::DiagonalMatrix<double, MS_SIZE> Q1;
  Eigen::DiagonalMatrix<double, MI_SIZE> Q2;
  Eigen::DiagonalMatrix<double, MY_SIZE> R;
  const MIPOParams param;
  void mipo_init_noise();
};
