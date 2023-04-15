/*
 * This file contains a set of functions for computation of the kinematics of
 * robot leg and some useful rotation related function, mainly euler angle and
 * quaternion conversion our euler angle is defined as roll-pitch-yaw (euler(0),
 * euler(1),euler(2)), which is the same as the one in ROS
 */

#pragma once
#include <torch/torch.h>

namespace legged {

// the skew symmetric matrix of a vector, input can be either
// torch::Tensor
torch::Tensor skewSymmetric(const torch::Tensor vec);

torch::Tensor euler_to_quat(const torch::Tensor euler);
torch::Tensor euler_to_rot(const torch::Tensor euler);

torch::Tensor mtx_w_to_euler_dot(const torch::Tensor euler);

torch::Tensor dyn_rk4(const torch::Tensor& xn, const torch::Tensor& un, const torch::Tensor& un1, double dt,
                      std::function<torch::Tensor(const torch::Tensor&, const torch::Tensor&)> dynfunc);

}  // namespace legged