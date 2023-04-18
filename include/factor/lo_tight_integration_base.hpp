#pragma once

#include <Eigen/Dense>

#define LO_TIGHT_RESIDUAL_SIZE 16
#define LO_TIGHT_NOISE_SIZE 16
#define RHO_SIZE 1
typedef Eigen::Matrix<double, RHO_SIZE, 1> Vec_rho;

class LOTightIntegrationBase {
 public:
  LOTightIntegrationBase() = delete;

  LOTightIntegrationBase(const int _leg_id, const Eigen::Vector3d& _body_gyr_0, const Eigen::Vector3d& _foot_gyr_0,
                         const Eigen::Vector3d& _jang_0, const Eigen::Vector3d& _jvel_0, const Eigen::Vector3d& _linearized_bg,
                         const Eigen::Vector3d& _linearized_bf, const Eigen::Vector3d& _linearized_bv)
      : linearized_body_gyr{_body_gyr_0},
        linearized_foot_gyr{_foot_gyr_0},
        linearized_jang{_jang_0},
        linearized_jvel{_jvel_0},
        linearized_bg{_linearized_bg},
        linearized_bf{_linearized_bf},
        linearized_bv{_linearized_bv} {
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

  void repropagate(const Eigen::Vector3d& _linearized_bg, const Eigen::Vector3d& _linearized_bf, const Eigen::Vector3d& _linearized_bv) {
    sum_dt = 0.0;
    body_gyr_0 = linearized_body_gyr;
    foot_gyr_0 = linearized_foot_gyr;
    jang_0 = linearized_jang;
    jvel_0 = linearized_jvel;

    delta_epsilon.setZero();
    delta_q.setIdentity();
    linearized_bg = _linearized_bg;
    linearized_bf = _linearized_bf;
    linearized_bv = _linearized_bv;
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
                           const Eigen::Vector3d& linearized_bg, const Eigen::Vector3d& linearized_bf,
                           const Eigen::Vector3d& linearized_bv,                                       // previous biases
                           Eigen::Vector3d& result_delta_epsilon, Eigen::Quaterniond& result_delta_q,  // result integration terms
                           Eigen::Vector3d& result_linearized_bg, Eigen::Vector3d& result_linearized_bf,
                           Eigen::Vector3d& result_linearized_bv,  // result biases
                           bool update_jacobian) {
    // orientation integration
    Eigen::Vector3d un_gyr = 0.5 * (_body_gyr_0 + _body_gyr_1) - linearized_bg;
    result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
    result_linearized_bg = linearized_bg;

    // LO velocity integration
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
  }

  Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, 1> IMULegIntegrationBase::evaluate(
      const Vector3d& Pi, const Quaterniond& Qi, const Vector3d& Vi, const Vector3d& Bai, const Vector3d& Bgi, const Vector3d& Bfi,
      const Vec_rho& rhoi, const Vector3d& Pj, const Quaterniond& Qj, const Vector3d& Vj, const Vector3d& Baj, const Vector3d& Bgj,
      const Vector3d& Bfj, const Vec_rho& rhoj) {
    Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, 1> residuals;
    residuals.setZero();

    return residuals;
  }

  double sum_dt;
  Eigen::Vector3d delta_epsilon;
  Eigen::Quaterniond delta_q;
  Eigen::Matrix<double, LO_TIGHT_RESIDUAL_SIZE, LO_TIGHT_RESIDUAL_SIZE> jacobian, covariance;

 private:
  int leg_id;  // 0 FL, 1 FR, 2 RL, 3 RR, very important when calling LOTightFactorUtils
  double dt;
  Eigen::Vector3d body_gyr_0, body_gyr_1;
  Eigen::Vector3d foot_gyr_0, foot_gyr_1;
  Eigen::Vector3d jang_0, jang_1;
  Eigen::Vector3d jvel_0, jvel_1;

  std::vector<double> dt_buf;
  std::vector<Eigen::Vector3d> body_gyr_buf;
  std::vector<Eigen::Vector3d> foot_gyr_buf;
  std::vector<Eigen::Vector3d> jang_buf;
  std::vector<Eigen::Vector3d> jvel_buf;

  const Eigen::Vector3d linearized_body_gyr, linearized_foot_gyr;
  const Eigen::Vector3d linearized_jang, linearized_foot_jvel;
  Eigen::Vector3d linearized_bg, linearized_bf, linearized_bv;

  Eigen::Matrix<double, LO_TIGHT_NOISE_SIZE, LO_TIGHT_NOISE_SIZE> noise;
};