#pragma once
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <torch/torch.h>
#include <vector>

class MIPOEstimatorTensor {
public:
  MIPOEstimatorTensor();
  ~MIPOEstimatorTensor();
  // process dynamics - torch tensor - tensor related functions are incomplete
  torch::Tensor mipo_process_dyn_tensor(const torch::Tensor &x,
                                        const torch::Tensor &u);
  torch::Tensor mipo_xk1(const torch::Tensor &x, const torch::Tensor &u0,
                         const torch::Tensor &u1, double dt);
};