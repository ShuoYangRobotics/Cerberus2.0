#include "mipo/MIPOEstimatorTensor.hpp"
#include "utils/torch_kino.hpp"
#include "utils/utils.hpp"

MIPOEstimatorTensor::MIPOEstimatorTensor() {}

MIPOEstimatorTensor::~MIPOEstimatorTensor() {}

torch::Tensor
MIPOEstimatorTensor::mipo_process_dyn_tensor(const torch::Tensor &x,
                                             const torch::Tensor &u) {
  torch::Tensor pos = x.slice(0, 0, 3);
  torch::Tensor vel = x.slice(0, 3, 6);
  torch::Tensor euler = x.slice(0, 6, 9);

  torch::Tensor foot1_pos = x.slice(0, 9, 12);
  torch::Tensor foot1_vel = x.slice(0, 12, 15);
  torch::Tensor foot2_pos = x.slice(0, 15, 18);
  torch::Tensor foot2_vel = x.slice(0, 18, 21);
  torch::Tensor foot3_pos = x.slice(0, 21, 24);
  torch::Tensor foot3_vel = x.slice(0, 24, 27);
  torch::Tensor foot4_pos = x.slice(0, 27, 30);
  torch::Tensor foot4_vel = x.slice(0, 30, 33);

  torch::Tensor ba = x.slice(0, 33, 36); // body acc bias
  torch::Tensor bg = x.slice(0, 36, 39); // gyro bias

  torch::Tensor foot1_ba = x.slice(0, 39, 42); // foot 1 acc bias
  torch::Tensor foot2_ba = x.slice(0, 42, 45); // foot 2 acc bias
  torch::Tensor foot3_ba = x.slice(0, 45, 48); // foot 3 acc bias
  torch::Tensor foot4_ba = x.slice(0, 48, 51); // foot 4 acc bias

  torch::Tensor w = u.slice(0, 0, 3) - bg; // body angular velocity
  torch::Tensor a = u.slice(0, 3, 6) - ba; // body linear acceleration

  torch::Tensor foot1_acc =
      u.slice(0, 6, 9) - foot1_ba; // already changed to body frame
  torch::Tensor foot2_acc =
      u.slice(0, 9, 12) - foot2_ba; // already changed to body frame
  torch::Tensor foot3_acc =
      u.slice(0, 12, 15) - foot3_ba; // already changed to body frame
  torch::Tensor foot4_acc =
      u.slice(0, 15, 18) - foot4_ba; // already changed to body frame

  torch::Tensor deuler = legged::mtx_w_to_euler_dot(euler).matmul(w);

  torch::Tensor R = legged::euler_to_rot(euler);

  // gravity 0, 0, 9.8
  torch::Tensor gravity = torch::zeros({3});
  gravity[0] = 0;
  gravity[1] = 0;
  gravity[2] = 9.8;

  torch::Tensor acc = R.matmul(a) - gravity;

  torch::Tensor foot1_acc_w = R.matmul(foot1_acc) - gravity;
  torch::Tensor foot2_acc_w = R.matmul(foot2_acc) - gravity;
  torch::Tensor foot3_acc_w = R.matmul(foot3_acc) - gravity;
  torch::Tensor foot4_acc_w = R.matmul(foot4_acc) - gravity;

  torch::Tensor xdot =
      torch::cat({vel, acc, deuler, foot1_vel, foot1_acc_w, foot2_vel,
                  foot2_acc_w, foot3_vel, foot3_acc_w, foot4_vel, foot4_acc_w,
                  torch::zeros({3}), // body acc bias
                  torch::zeros({3}), // gyro bias
                  torch::zeros({3}), // foot 1 acc bias
                  torch::zeros({3}), // foot 2 acc bias
                  torch::zeros({3}), // foot 3 acc bias
                  torch::zeros({3}), // foot 4 acc bias
                  torch::ones({1}) * 1.0});

  return xdot;
}

torch::Tensor MIPOEstimatorTensor::mipo_xk1(const torch::Tensor &x,
                                            const torch::Tensor &u0,
                                            const torch::Tensor &u1,
                                            double dt) {
  // use rk4 function to get xk+1
  // pass MIPOEstimatorTensor::mipo_process_dyn_tensor as a std::function
  // pointer
  return legged::dyn_rk4(
      x, u0, u1, dt,
      std::bind(&MIPOEstimatorTensor::mipo_process_dyn_tensor, this,
                std::placeholders::_1, std::placeholders::_2));
}
