/*
 * This file contains a set of functions for computation of the kinematics of
 * robot leg and some useful rotation related function, mainly euler angle and
 * quaternion conversion our euler angle is defined as roll-pitch-yaw (euler(0),
 * euler(1),euler(2)), which is the same as the one in ROS
 *  we use eigen and casadi for computation
 */
#pragma once
#include <Eigen/Dense>
#include <casadi/casadi.hpp>
// the operator<< in casadi_misc.hpp contradict with the same overloaded
// function in libtorch library

namespace legged {

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> skewSymmetric(Eigen::Matrix<SCALAR_T, 3, 1> vec) {
  Eigen::Matrix<SCALAR_T, 3, 3> skewSymmetricMatrix(3, 3);
  skewSymmetricMatrix << SCALAR_T(0.0), -vec(2), vec(1), vec(2), SCALAR_T(0.0),
      -vec(0), -vec(1), vec(0), SCALAR_T(0.0);

  return skewSymmetricMatrix;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 4, 1>
euler_to_quat(const Eigen::Matrix<SCALAR_T, 3, 1> &euler) {
  SCALAR_T roll = euler(0);
  SCALAR_T pitch = euler(1);
  SCALAR_T yaw = euler(2);

  yaw = yaw / 2.0;
  pitch = pitch / 2.0;
  roll = roll / 2.0;

  SCALAR_T cos_half_yaw = cos(yaw);
  SCALAR_T sin_half_yaw = sin(yaw);
  SCALAR_T cos_half_pitch = cos(pitch);
  SCALAR_T sin_half_pitch = sin(pitch);
  SCALAR_T cos_half_roll = cos(roll);
  SCALAR_T sin_half_roll = sin(roll);

  Eigen::Matrix<SCALAR_T, 4, 1> quat;
  quat << cos_half_roll * cos_half_pitch * cos_half_yaw +
              sin_half_roll * sin_half_pitch * sin_half_yaw,
      sin_half_roll * cos_half_pitch * cos_half_yaw -
          cos_half_roll * sin_half_pitch * sin_half_yaw,
      cos_half_roll * sin_half_pitch * cos_half_yaw +
          sin_half_roll * cos_half_pitch * sin_half_yaw,
      cos_half_roll * cos_half_pitch * sin_half_yaw -
          sin_half_roll * sin_half_pitch * cos_half_yaw;

  return quat;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3>
euler_to_rot(const Eigen::Matrix<SCALAR_T, 3, 1> &euler) {
  SCALAR_T r = euler(0);
  SCALAR_T p = euler(1);
  SCALAR_T y = euler(2);

  Eigen::Matrix<SCALAR_T, 3, 3> Ry, Rp, Rr;
  Ry << cos(y), -sin(y), 0, sin(y), cos(y), 0, 0, 0, 1;

  Rp << cos(p), 0, sin(p), 0, 1, 0, -sin(p), 0, cos(p);

  Rr << 1, 0, 0, 0, cos(r), -sin(r), 0, sin(r), cos(r);

  Eigen::Matrix<SCALAR_T, 3, 3> R = Ry * Rp * Rr;
  return R;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3>
mtx_w_to_euler_dot(const Eigen::Matrix<SCALAR_T, 3, 1> &euler) {
  SCALAR_T r = euler(0);
  SCALAR_T p = euler(1);
  SCALAR_T y = euler(2);

  SCALAR_T sr = sin(r);
  SCALAR_T sp = sin(p);
  SCALAR_T cr = cos(r);
  SCALAR_T cp = cos(p);

  Eigen::Matrix<SCALAR_T, 3, 3> mtx;
  mtx << 1, (sr * sp) / cp, (cr * sp) / cp, 0, cr, -sr, 0, sr / cp, cr / cp;

  return mtx;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1>
quat_to_euler(const Eigen::Quaternion<SCALAR_T> &quat) {
  SCALAR_T w = quat.w();
  SCALAR_T x = quat.x();
  SCALAR_T y = quat.y();
  SCALAR_T z = quat.z();

  SCALAR_T cos_pitch_cos_yaw = 1.0 - 2.0 * (y * y + z * z);
  SCALAR_T cos_pitch_sin_yaw = 2.0 * (x * y + w * z);
  SCALAR_T sin_pitch = -2.0 * (x * z - w * y);
  SCALAR_T cos_pitch;
  SCALAR_T sin_roll_cos_pitch = 2.0 * (y * z + w * x);
  SCALAR_T cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y);

  cos_pitch = std::sqrt(cos_pitch_cos_yaw * cos_pitch_cos_yaw +
                        cos_pitch_sin_yaw * cos_pitch_sin_yaw);

  SCALAR_T yaw = std::atan2(cos_pitch_sin_yaw, cos_pitch_cos_yaw);
  SCALAR_T pitch = std::abs(sin_pitch) >= 1 ? std::copysign(M_PI / 2, sin_pitch)
                                            : std::asin(sin_pitch);
  SCALAR_T roll = std::atan2(sin_roll_cos_pitch, cos_roll_cos_pitch);

  Eigen::Matrix<SCALAR_T, 3, 1> euler(roll, pitch, yaw);
  return euler;
}

template <typename SCALAR_T, int size_x, int size_u>
Eigen::Matrix<SCALAR_T, size_x, 1>
dyn_rk4_casadi(const Eigen::Matrix<SCALAR_T, size_x, 1> &xn,
               const Eigen::Matrix<SCALAR_T, size_u, 1> &un,
               const Eigen::Matrix<SCALAR_T, size_u, 1> &un1, SCALAR_T dt,
               std::function<Eigen::Matrix<SCALAR_T, size_x, 1>(
                   const Eigen::Matrix<SCALAR_T, size_x, 1> &,
                   const Eigen::Matrix<SCALAR_T, size_u, 1> &)>
                   dynfunc) {
  Eigen::Matrix<SCALAR_T, size_x, 1> k1 = dynfunc(xn, un);
  Eigen::Matrix<SCALAR_T, size_x, 1> k2 =
      dynfunc(xn + dt * k1 / 2, (un + un1) / 2);
  Eigen::Matrix<SCALAR_T, size_x, 1> k3 =
      dynfunc(xn + dt * k2 / 2, (un + un1) / 2);
  Eigen::Matrix<SCALAR_T, size_x, 1> k4 = dynfunc(xn + dt * k3, un1);

  Eigen::Matrix<SCALAR_T, size_x, 1> xn1 =
      xn + (1 / 6.0) * dt * (k1 + 2 * k2 + 2 * k3 + k4);
  return xn1;
}

// the kinematic functions of unitree robot goes here
// https://github.com/ShuoYangRobotics/Multi-IMU-Proprioceptive-Odometry/blob/main/kinematics_init_lc.m
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1>
fk_pf_pos(const Eigen::Matrix<SCALAR_T, 3, 1> j_pos,
          const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> param) {
  auto ox = param(0);
  auto oy = param(1);
  auto d = param(2);
  auto lt = param(3);
  auto lc = param(4);
  auto t1 = j_pos(0);
  auto t2 = j_pos(1);
  auto t3 = j_pos(2);
  // fk_pf_pos = ox - lc *sin(t2 + t3) - lt *sin(t2),
  // oy + d *cos(t1) + lt *cos(t2) * sin(t1) +
  // lc *cos(t2) * cos(t3) * sin(t1) -
  // lc *sin(t1) * sin(t2) * sin(t3),
  // d *sin(t1) - lt *cos(t1) * cos(t2) -
  // lc *cos(t1) * cos(t2) * cos(t3) + lc *cos(t1) * sin(t2) * sin(t3)
  Eigen::Matrix<SCALAR_T, 3, 1> fk_pf_pos;
  fk_pf_pos << ox - lc * sin(t2 + t3) - lt * sin(t2),
      oy + d * cos(t1) + lt * cos(t2) * sin(t1) +
          lc * cos(t2) * cos(t3) * sin(t1) - lc * sin(t1) * sin(t2) * sin(t3),
      d * sin(t1) - lt * cos(t1) * cos(t2) - lc * cos(t1) * cos(t2) * cos(t3) +
          lc * cos(t1) * sin(t2) * sin(t3);

  return fk_pf_pos;
}
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3>
fk_pf_rot(const Eigen::Matrix<SCALAR_T, 3, 1> j_pos) {
  auto t1 = j_pos(0);
  auto t2 = j_pos(1);
  auto t3 = j_pos(2);
  Eigen::Matrix<SCALAR_T, 3, 3> fk_pf_rot;
  fk_pf_rot << cos(t2 + t3), 0, sin(t2 + t3), sin(t2 + t3) * sin(t1), cos(t1),
      -cos(t2 + t3) * sin(t1), -sin(t2 + t3) * cos(t1), sin(t1),
      cos(t2 + t3) * cos(t1);

  return fk_pf_rot;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3>
d_fk_dt(const Eigen::Matrix<SCALAR_T, 3, 1> j_pos,
        const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> param) {
  auto ox = param(0);
  auto oy = param(1);
  auto d = param(2);
  auto lt = param(3);
  auto lc = param(4);
  auto t1 = j_pos(0);
  auto t2 = j_pos(1);
  auto t3 = j_pos(2);

  Eigen::Matrix<SCALAR_T, 3, 3> d_fk_dt;
  d_fk_dt << 0, -lc * cos(t2 + t3) - lt * cos(t2), -lc * cos(t2 + t3),
      lt * cos(t1) * cos(t2) - d * sin(t1) + lc * cos(t1) * cos(t2) * cos(t3) -
          lc * cos(t1) * sin(t2) * sin(t3),
      -sin(t1) * (lc * sin(t2 + t3) + lt * sin(t2)),
      -lc * sin(t2 + t3) * sin(t1),
      d * cos(t1) + lt * cos(t2) * sin(t1) + lc * cos(t2) * cos(t3) * sin(t1) -
          lc * sin(t1) * sin(t2) * sin(t3),
      cos(t1) * (lc * sin(t2 + t3) + lt * sin(t2)), lc * sin(t2 + t3) * cos(t1);

  return d_fk_dt;
}

} // namespace legged