#include <iostream>

#include "factor/lo_tight_integration_base.hpp"
#include "utils/LOTightUtils.hpp"

LOTightUtils* tightUtils = new LOTightUtils();

LOTightIntegrationBase* lo_pre_integration;
int main() {
  std::cout << "Hello, World!" << std::endl;

  Eigen::Vector3d jang = {0.190146958330476, 0.467755352958025, -1.470605609393327};
  Eigen::Vector3d jvel = {-0.062272018923413, 0.294882247382246, 0.203360327979819};
  Eigen::Vector3d body_gyr = {0.122406622908672, 0.159735927477272, -0.049865507506960};
  Eigen::Vector3d foot_gyr = {0.071829827864751, 0.212580560947972, -0.646583192503642};
  Eigen::Vector3d linearized_bg = {-0.003610757106939, 0.007919617761550, -0.003208871191900};
  Eigen::Vector3d linearized_bf = {0, 0, 0};
  Eigen::Vector3d linearized_bv = {0, 0, 0};
  double d0 = 0.05;
  Vec_rho rho;
  rho(0) = d0;

  lo_pre_integration =
      new LOTightIntegrationBase(0, jang, jvel, body_gyr, foot_gyr, linearized_bg, linearized_bf, linearized_bv, rho, tightUtils);

  lo_pre_integration->push_back(0.5, body_gyr, foot_gyr, jang, jvel);
  std::cout << lo_pre_integration->covariance.diagonal().transpose() << std::endl;
  std::cout << "  --  " << std::endl;
  lo_pre_integration->push_back(0.5, body_gyr, foot_gyr, jang, jvel);
  std::cout << lo_pre_integration->covariance.diagonal().transpose() << std::endl;
  std::cout << "  --  " << std::endl;
  lo_pre_integration->push_back(0.5, body_gyr, foot_gyr, jang, jvel);
  std::cout << lo_pre_integration->covariance.diagonal().transpose() << std::endl;
  std::cout << "  --  " << std::endl;
  lo_pre_integration->push_back(0.5, body_gyr, foot_gyr, jang, jvel);
  std::cout << lo_pre_integration->covariance.diagonal().transpose() << std::endl;
  return 0;
}