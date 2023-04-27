#include <iostream>

#include "utils/LOTightUtils.hpp"

LOTightUtils* tightUtils = new LOTightUtils();

int main(int argc, char** argv) {
  std::cout << "Hello, world!" << std::endl;
  Eigen::Vector3d jang = {0.190146958330476, 0.467755352958025, -1.470605609393327};
  Eigen::Vector3d jvel = {-0.062272018923413, 0.294882247382246, 0.203360327979819};
  Eigen::Vector3d body_gyr = {0.122406622908672, 0.159735927477272, -0.049865507506960};
  Eigen::Vector3d foot_gyr = {0.071829827864751, 0.212580560947972, -0.646583192503642};
  Eigen::Vector3d linearized_bg = {-0.003610757106939, 0.007919617761550, -0.003208871191900};
  Eigen::Vector3d linearized_bf = {0, 0, 0};
  Eigen::Vector3d linearized_bv = {0, 0, 0};
  double d0 = 0.05;

  // body velocity calculation notice that the body velocity is in the body frame
  Eigen::Vector3d body_v = tightUtils->calBodyVel(0, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, d0);
  std::cout << body_v << std::endl;

  // from matlab
  Eigen::Vector3d body_v_true = {0.158250918118447, -0.018625002145291, 0.098871071395940};
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