#include <iostream>

#include "utils/LOTightUtils.hpp"

LOTightUtils* tightUtils = new LOTightUtils();

int main(int argc, char** argv) {
  std::cout << "Hello, world!" << std::endl;
  Eigen::Vector3d jang = {1, 1, 1};
  Eigen::Vector3d jvel = {1, 1, 1};
  Eigen::Vector3d body_gyr = {1, 1, 1};
  Eigen::Vector3d foot_gyr = {1, 1, 1};
  Eigen::Vector3d linearized_bg = {1, 1, 1};
  Eigen::Vector3d linearized_bf = {1, 1, 1};
  Eigen::Vector3d linearized_bv = {1, 1, 1};
  double d0 = 0.05;

  // body velocity calculation
  Eigen::Vector3d body_v = tightUtils->calBodyVel(0, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);
  std::cout << body_v << std::endl;
  // derivatives
  Eigen::MatrixXd dv_djvel;
  return 0;
}