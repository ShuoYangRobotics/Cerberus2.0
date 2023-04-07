#include "utils/parameters.hpp"

/*
 * PO message queue
 */
int MIN_PO_QUEUE_SIZE = 10;
int MAX_PO_QUEUE_SIZE = 150;

int FOOT_IMU_MOVMEAN_WINDOW_SIZE = 10;
int JOINT_MOVMEAN_WINDOW_SIZE = 10;

// topic names
std::string IMU_TOPIC;
std::string JOINT_FOOT_TOPIC;
std::string FL_IMU_TOPIC;
std::string FR_IMU_TOPIC;
std::string RL_IMU_TOPIC;
std::string RR_IMU_TOPIC;
std::string GT_TOPIC;
std::string CAMERA0_TOPIC;
std::string CAMERA1_TOPIC;

// this variable means the estimator actually always estimates the state at the
// current time - LAG_TIME. We do so to account for potential delays and sensor
// information mistaches
double LAG_TIME = 0.0;  // 100ms

double FOOT_IMU_DELAY = 0.0;  // 23ms, this is estimated from analysing data in Matlab

/*
 * VINS Fusion parameters
 */
// const double FOCAL_LENGTH = 460.0;
// const int WINDOW_SIZE = 10;
// const int NUM_OF_F = 1000;
//#define UNIT_SPHERE_ERROR

double INIT_DEPTH;
double MIN_PARALLAX;
int ESTIMATE_EXTRINSIC;

double ACC_N, ACC_N_Z, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;  // num of cam, imu to camera rotation
std::vector<Eigen::Vector3d> TIC;  // num of cam, imu to camera position
Eigen::Vector3d G;

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
std::string EX_CALIB_RESULT_PATH;
std::string DATASET_NAME;
std::string VILO_RESULT_PATH;
std::string OUTPUT_FOLDER;
double TD;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
int ROW, COL;
int NUM_OF_CAM;
int STEREO;
int USE_IMU;

std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;

void Utils::readParameters(ros::NodeHandle& nh_) {
  // default values
  IMU_TOPIC = "/unitree_hardware/imu";
  JOINT_FOOT_TOPIC = "/unitree_hardware/joint_foot";
  FL_IMU_TOPIC = "/WT901_49_Data";
  FR_IMU_TOPIC = "/WT901_48_Data";
  RL_IMU_TOPIC = "/WT901_50_Data";
  RR_IMU_TOPIC = "/WT901_47_Data";
  GT_TOPIC = "/mocap_node/Go1_body/pose";
  CAMERA0_TOPIC = "/camera_forward/infra1/image_rect_raw";
  CAMERA1_TOPIC = "/camera_forward/infra2/image_rect_raw";

  // read from rosparam
  // read parameters from ros server
  nh_.param<std::string>("IMU_TOPIC", IMU_TOPIC, "/unitree_hardware/imu");
  nh_.param<std::string>("JOINT_FOOT_TOPIC", JOINT_FOOT_TOPIC, "/unitree_hardware/joint_foot");
  nh_.param<std::string>("FL_IMU_TOPIC", FL_IMU_TOPIC, "/WT901_49_Data");
  nh_.param<std::string>("FR_IMU_TOPIC", FR_IMU_TOPIC, "/WT901_48_Data");
  nh_.param<std::string>("RL_IMU_TOPIC", RL_IMU_TOPIC, "/WT901_50_Data");
  nh_.param<std::string>("RR_IMU_TOPIC", RR_IMU_TOPIC, "/WT901_47_Data");
  nh_.param<std::string>("GT_TOPIC", GT_TOPIC, "/mocap_node/Go1_body/pose");
}
