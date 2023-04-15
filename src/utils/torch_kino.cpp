#include "utils/torch_kino.hpp"

namespace legged {

torch::Tensor skewSymmetric(const torch::Tensor vec) {
  torch::Tensor skewSymmetricMatrix = torch::zeros({3, 3});
  skewSymmetricMatrix[0][1] = -vec[2][0];
  skewSymmetricMatrix[0][2] = vec[1][0];
  skewSymmetricMatrix[1][0] = vec[2][0];
  skewSymmetricMatrix[1][2] = -vec[0][0];
  skewSymmetricMatrix[2][0] = -vec[1][0];
  skewSymmetricMatrix[2][1] = vec[0][0];

  return skewSymmetricMatrix;
}

// euler to quat
torch::Tensor euler_to_quat(const torch::Tensor euler) {
  // Convert roll-pitch-yaw Euler angles to quaternion
  // Input:
  //   euler - input tensor containing roll, pitch, yaw Euler angles
  // Output:
  //   quat - output tensor containing the resulting quaternion

  auto roll = euler[0] / 2.0;
  auto pitch = euler[1] / 2.0;
  auto yaw = euler[2] / 2.0;

  auto cos_half_yaw = torch::cos(yaw);
  auto sin_half_yaw = torch::sin(yaw);
  auto cos_half_pitch = torch::cos(pitch);
  auto sin_half_pitch = torch::sin(pitch);
  auto cos_half_roll = torch::cos(roll);
  auto sin_half_roll = torch::sin(roll);

  torch::Tensor quat = torch::zeros({4});
  quat[0] = cos_half_roll * cos_half_pitch * cos_half_yaw + sin_half_roll * sin_half_pitch * sin_half_yaw;
  quat[1] = sin_half_roll * cos_half_pitch * cos_half_yaw - cos_half_roll * sin_half_pitch * sin_half_yaw;
  quat[2] = cos_half_roll * sin_half_pitch * cos_half_yaw + sin_half_roll * cos_half_pitch * sin_half_yaw;
  quat[3] = cos_half_roll * cos_half_pitch * sin_half_yaw - sin_half_roll * sin_half_pitch * cos_half_yaw;

  return quat;
}

torch::Tensor euler_to_rot(const torch::Tensor euler) {
  // Convert roll-pitch-yaw Euler angles to rotation matrix
  // Input:
  //   euler - input tensor containing roll, pitch, yaw Euler angles
  // Output:
  //   R - output tensor containing the resulting rotation matrix

  auto r = euler[0];
  auto p = euler[1];
  auto y = euler[2];

  /*
  Ry = [cos(y) -sin(y) 0;
      sin(y) cos(y)  0;
      0    0  1];
  */
  torch::Tensor Ry = torch::empty({3, 3});
  Ry[0][0] = torch::cos(y);
  Ry[0][1] = -torch::sin(y);
  Ry[0][2] = 0.0;
  Ry[1][0] = torch::sin(y);
  Ry[1][1] = torch::cos(y);
  Ry[1][2] = 0.0;
  Ry[2][0] = 0.0;
  Ry[2][1] = 0.0;
  Ry[2][2] = 1.0;
  /*
    Rp = [cos(p)  0  sin(p);
        0   1   0;
        -sin(p)  0 cos(p)];
  */
  torch::Tensor Rp = torch::empty({3, 3});
  Rp[0][0] = torch::cos(p);
  Rp[0][1] = 0.0;
  Rp[0][2] = torch::sin(p);
  Rp[1][0] = 0.0;
  Rp[1][1] = 1.0;
  Rp[1][2] = 0.0;
  Rp[2][0] = -torch::sin(p);
  Rp[2][1] = 0.0;
  Rp[2][2] = torch::cos(p);
  /*
    Rr = [1  0  0 ;
        0  cos(r)  -sin(r);
        0  sin(r)   cos(r)];
  */
  torch::Tensor Rr = torch::empty({3, 3});
  Rr[0][0] = 1.0;
  Rr[0][1] = 0.0;
  Rr[0][2] = 0.0;
  Rr[1][0] = 0.0;
  Rr[1][1] = torch::cos(r);
  Rr[1][2] = -torch::sin(r);
  Rr[2][0] = 0.0;
  Rr[2][1] = torch::sin(r);
  Rr[2][2] = torch::cos(r);

  torch::Tensor R = torch::mm(Ry, Rp);
  R = torch::mm(R, Rr);

  return R;
}

torch::Tensor mtx_w_to_euler_dot(const torch::Tensor euler) {
  auto r = euler[0];
  auto p = euler[1];
  auto y = euler[2];

  torch::Tensor mtx = torch::zeros({3, 3});

  mtx[0][0] = 1.0;
  mtx[0][1] = (sin(p) * sin(r)) / cos(p);
  mtx[0][2] = (cos(r) * sin(p)) / cos(p);
  mtx[1][0] = 0.0;
  mtx[1][1] = cos(r);
  mtx[1][2] = -sin(r);
  mtx[2][0] = 0.0;
  mtx[2][1] = sin(r) / cos(p);
  mtx[2][2] = cos(r) / cos(p);

  return mtx;
}

torch::Tensor dyn_rk4(const torch::Tensor& xn, const torch::Tensor& un, const torch::Tensor& un1, double dt,
                      std::function<torch::Tensor(const torch::Tensor&, const torch::Tensor&)> dynfunc) {
  torch::Tensor k1 = dynfunc(xn, un);
  torch::Tensor k2 = dynfunc(xn + dt * k1 / 2, (un + un1) / 2);
  torch::Tensor k3 = dynfunc(xn + dt * k2 / 2, (un + un1) / 2);
  torch::Tensor k4 = dynfunc(xn + dt * k3, un1);

  torch::Tensor xn1 = xn + (1 / 6.0) * dt * (k1 + 2 * k2 + 2 * k3 + k4);
  return xn1;
}

}  // namespace legged