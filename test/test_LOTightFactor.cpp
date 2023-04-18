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
  Eigen::MatrixXd dv_djang =
      tightUtils->calBodyVelDjang(0, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);
  std::cout << dv_djang << std::endl;

  Eigen::MatrixXd dv_djvel =
      tightUtils->calBodyVelDjvel(0, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);
  std::cout << dv_djvel << std::endl;

  Eigen::MatrixXd dv_dw =
      tightUtils->calBodyVelDbodyGyr(0, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);
  std::cout << dv_dw << std::endl;

  Eigen::MatrixXd dv_dwf =
      tightUtils->calBodyVelDfootGyr(0, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);
  std::cout << dv_dwf << std::endl;

  Eigen::MatrixXd dv_dbg = tightUtils->calBodyVelDbg(0, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);
  std::cout << dv_dbg << std::endl;

  Eigen::MatrixXd dv_dbf = tightUtils->calBodyVelDbf(0, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);
  std::cout << dv_dbf << std::endl;

  Eigen::MatrixXd dv_dbv = tightUtils->calBodyVelDbv(0, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);
  std::cout << dv_dbv << std::endl;

  Eigen::MatrixXd dv_dd0 = tightUtils->calBodyVelDd0(0, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);
  std::cout << dv_dd0 << std::endl;
  return 0;
}