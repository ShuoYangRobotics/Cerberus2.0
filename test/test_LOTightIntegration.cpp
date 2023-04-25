#include <iostream>

#include "factor/lo_tight_factor.hpp"
#include "factor/lo_tight_integration_base.hpp"
#include "utils/LOTightUtils.hpp"

LOTightUtils* tightUtils = new LOTightUtils();

LOTightIntegrationBase* lo_pre_integration;
int main() {
  std::string config_file = "/home/EstimationUser/estimation_ws/src/cerberus2/config/go1_config/hardware_go1_vilo_config.yaml";
  Utils::readParametersFile(config_file);

  std::cout << "Hello, World!" << std::endl;
  // code gen

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

  LOTightFactor* lo_tight_factor = new LOTightFactor(lo_pre_integration);

  double para_Pose[WINDOW_SIZE + 1][7] = {0};
  for (int i = 0; i < WINDOW_SIZE + 1; i++) {
    para_Pose[i][0] = 0.01 * i;
    para_Pose[i][1] = 0.01 * i;
    para_Pose[i][2] = 0.01 * i;
    para_Pose[i][3] = 0;
    para_Pose[i][4] = 0;
    para_Pose[i][5] = 0;
    para_Pose[i][6] = 1;
  }
  double para_SpeedBias[WINDOW_SIZE + 1][9] = {0};
  for (int i = 0; i < WINDOW_SIZE + 1; i++) {
    for (int j = 0; j < 9; j++) para_SpeedBias[i][j] = 0.01 * i;
  }
  double para_FootBias[WINDOW_SIZE + 1][4][7] = {0};
  for (int i = 0; i < WINDOW_SIZE + 1; i++) {
    for (int j = 0; j < 4; j++) {
      for (int k = 0; k < 7; k++) {
        para_FootBias[i][j][k] = 0.01 * i + 0.01 * k;
      }
    }
  }

  int i = 3;
  int j = i + 1;
  std::vector<double*> parameter_blocks =
      std::vector<double*>{para_Pose[i], para_SpeedBias[i], para_FootBias[i][0], para_Pose[j], para_SpeedBias[j], para_FootBias[j][0]};
  std::vector<int> block_sizes = lo_tight_factor->parameter_block_sizes();
  Eigen::VectorXd residuals;
  residuals.resize(lo_tight_factor->num_residuals());
  double** raw_jacobians = new double*[block_sizes.size()];
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
  jacobians.resize(block_sizes.size());
  for (int xx = 0; xx < static_cast<int>(block_sizes.size()); xx++) {
    jacobians[xx].resize(lo_tight_factor->num_residuals(), block_sizes[xx]);
    raw_jacobians[xx] = jacobians[xx].data();
    // dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
  }
  lo_tight_factor->Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians);
  // std::cout << "residual between frame " << i << " and " << j << std::endl;
  // std::cout << residuals.transpose() << std::endl;
  lo_tight_factor->checkJacobian(parameter_blocks.data());

  return 0;
}