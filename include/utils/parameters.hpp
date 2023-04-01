#pragma once
#include <Eigen/Dense>

// robot constant
#define NUM_LEG 4
#define NUM_DOF_PER_LEG 3
#define DIM_GRF 12
#define DIM_GRF_PER_LEG 3
#define DIM_GRF 12
#define NUM_DOF 12

int WINDOW_SIZE = 100;
int MIN_WINDOW_SIZE = 10;
int FOOT_IMU_WINDOW_SIZE = 70;
double WINDOW_TIME_SPAN = 0.8; // 400ms
int MAX_MESSAGE_QUEUE_SIZE = 150;

int FOOT_IMU_MOVMEAN_WINDOW_SIZE = 10;
int JOINT_MOVMEAN_WINDOW_SIZE = 10;

std::string IMU_TOPIC;
std::string JOINT_FOOT_TOPIC;
std::string FL_IMU_TOPIC;
std::string FR_IMU_TOPIC;
std::string RL_IMU_TOPIC;
std::string RR_IMU_TOPIC;
std::string GT_TOPIC;

// this variable means the estimator actually always estimates the state at the
// current time - LAG_TIME. We do so to account for potential delays and sensor
// information mistaches
double LAG_TIME = 0.0; // 100ms

double FOOT_IMU_DELAY =
    0.0; // 23ms, this is estimated from analysing data in Matlab

namespace Utils {

void readParameters() {
  // default values
  IMU_TOPIC = "/unitree_hardware/imu";
  JOINT_FOOT_TOPIC = "/unitree_hardware/joint_foot";
  FL_IMU_TOPIC = "/WT901_49_Data";
  FR_IMU_TOPIC = "/WT901_48_Data";
  RL_IMU_TOPIC = "/WT901_50_Data";
  RR_IMU_TOPIC = "/WT901_47_Data";
  GT_TOPIC = "/iphone/gt";

  // read from rosparam
}
} // namespace Utils
