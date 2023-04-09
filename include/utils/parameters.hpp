#pragma once
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <string>

// robot constant
#define NUM_LEG 4
#define NUM_DOF_PER_LEG 3
#define DIM_GRF 12
#define DIM_GRF_PER_LEG 3
#define DIM_GRF 12
#define NUM_DOF 12

// size and state order definitions

enum SIZE_PARAMETERIZATION {
  SIZE_POSE = 7,      // position, quaternion
  SIZE_SPEEDBIAS = 9, // velocity, bias acc, bias gyr
  SIZE_FEATURE = 1
};

enum StateOrder { O_P = 0, O_R = 3, O_V = 6, O_BA = 9, O_BG = 12 };

enum NoiseOrder { O_AN = 0, O_GN = 3, O_AW = 6, O_GW = 9 };

/*
 * PO message queue
 */
extern int MIN_PO_QUEUE_SIZE;
extern int MAX_PO_QUEUE_SIZE;
extern int FOOT_IMU_MOVMEAN_WINDOW_SIZE;
extern int JOINT_MOVMEAN_WINDOW_SIZE;

// topic names
extern std::string IMU_TOPIC;
extern std::string LEG_TOPIC;
extern std::string FL_IMU_TOPIC;
extern std::string FR_IMU_TOPIC;
extern std::string RL_IMU_TOPIC;
extern std::string RR_IMU_TOPIC;
extern std::string GT_TOPIC;
extern std::string IMAGE0_TOPIC;
extern std::string IMAGE1_TOPIC;

// this variable means the estimator actually always estimates the state at the
// current time - LAG_TIME. We do so to account for potential delays and sensor
// information mistaches
extern double LAG_TIME; // 100ms

extern double
    FOOT_IMU_DELAY; // 23ms, this is estimated from analysing data in Matlab

/*
 * VINS Fusion parameters
 */
const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_F = 1000;
//#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_N_Z, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC; // num of cam, imu to camera rotation
extern std::vector<Eigen::Vector3d> TIC; // num of cam, imu to camera position
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string DATASET_NAME;
extern std::string VILO_RESULT_PATH;
extern std::string OUTPUT_FOLDER;
extern double TD;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern int ROW, COL;
extern int NUM_OF_CAM;
extern int STEREO;
extern int USE_IMU;

extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int FLOW_BACK;

namespace Utils {

void readParametersROS(ros::NodeHandle &nh_);
void readParametersFile(std::string config_file);

static std::string GetCurrentTimeForFileName() {
  auto time = std::time(nullptr);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time),
                      "%F_%T"); // ISO 8601 without timezone information.
  auto s = ss.str();
  std::replace(s.begin(), s.end(), ':', '-');
  return s;
}

} // namespace Utils