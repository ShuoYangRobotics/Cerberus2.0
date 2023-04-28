#include "utils/parameters.hpp"

/*
 * PO message queue
 */
int MIN_PO_QUEUE_SIZE = 25;
int MAX_PO_QUEUE_SIZE = 300;

int BODY_IMU_MOVMEAN_WINDOW_SIZE = 5;
int FOOT_IMU_MOVMEAN_WINDOW_SIZE = 5;
int JOINT_MOVMEAN_WINDOW_SIZE = 10;
int YAW_MOVMEAN_WINDOW_SIZE = 10;

// topic names
std::string IMU_TOPIC;
std::string LEG_TOPIC;
std::string FL_IMU_TOPIC;
std::string FR_IMU_TOPIC;
std::string RL_IMU_TOPIC;
std::string RR_IMU_TOPIC;
std::string GT_TOPIC;
std::string IMAGE0_TOPIC;
std::string IMAGE1_TOPIC;

// this variable means the estimator actually always estimates the state at the
// current time - LAG_TIME. We do so to account for potential delays and sensor
// information mistaches
double LAG_TIME = 0.0;  // 100ms

double FOOT_PRESSURE_DELAY = 0.03;  // 30ms this is the foot pressure sensor delay
double FOOT_IMU_DELAY = 0.023;      // 23ms, this is estimated from analysing data in Matlab

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
int VILO_FUSION_TYPE;
int KF_TYPE;

double ACC_N, ACC_N_Z, ACC_W;
double GYR_N, GYR_W;

double JOINT_ANG_N, JOINT_VEL_N;
double FOOT_GYR_N, FOOT_GYR_W, FOOT_VEL_W, RHO_W;

int ESTIMATE_KINEMATIC;

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
std::string PO_RESULT_PATH;
std::string GT_RESULT_PATH;
std::string OUTPUT_FOLDER;
double TD;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
int ROW, COL;
int NUM_OF_CAM;
int STEREO;

std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;

std::mutex casadi_mtx;

void Utils::readParametersROS(ros::NodeHandle& nh_) {
  // default values
  IMU_TOPIC = "/unitree_hardware/imu";
  LEG_TOPIC = "/unitree_hardware/joint_foot";
  FL_IMU_TOPIC = "/WT901_49_Data";
  FR_IMU_TOPIC = "/WT901_48_Data";
  RL_IMU_TOPIC = "/WT901_50_Data";
  RR_IMU_TOPIC = "/WT901_47_Data";
  GT_TOPIC = "/mocap_node/Go1_body/pose";
  IMAGE0_TOPIC = "/camera_forward/infra1/image_rect_raw";
  IMAGE1_TOPIC = "/camera_forward/infra2/image_rect_raw";

  // read from rosparam
  // read parameters from ros server
  nh_.param<std::string>("IMU_TOPIC", IMU_TOPIC, "/unitree_hardware/imu");
  nh_.param<std::string>("LEG_TOPIC", LEG_TOPIC, "/unitree_hardware/joint_foot");
  nh_.param<std::string>("FL_IMU_TOPIC", FL_IMU_TOPIC, "/WT901_49_Data");
  nh_.param<std::string>("FR_IMU_TOPIC", FR_IMU_TOPIC, "/WT901_48_Data");
  nh_.param<std::string>("RL_IMU_TOPIC", RL_IMU_TOPIC, "/WT901_50_Data");
  nh_.param<std::string>("RR_IMU_TOPIC", RR_IMU_TOPIC, "/WT901_47_Data");
  nh_.param<std::string>("GT_TOPIC", GT_TOPIC, "/mocap_node/Go1_body/pose");
}

void Utils::readParametersFile(std::string config_file) {
  FILE* fh = fopen(config_file.c_str(), "r");
  if (fh == NULL) {
    ROS_WARN("config_file dosen't exist; wrong config_file path");
    ROS_BREAK();
    return;
  }
  fclose(fh);

  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }
  fsSettings["dataset_name"] >> DATASET_NAME;

  fsSettings["image0_topic"] >> IMAGE0_TOPIC;
  fsSettings["image1_topic"] >> IMAGE1_TOPIC;
  MAX_CNT = fsSettings["max_cnt"];
  MIN_DIST = fsSettings["min_dist"];
  F_THRESHOLD = fsSettings["F_threshold"];
  SHOW_TRACK = fsSettings["show_track"];
  FLOW_BACK = fsSettings["flow_back"];

  fsSettings["imu_topic"] >> IMU_TOPIC;
  printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());

  fsSettings["leg_topic"] >> LEG_TOPIC;
  printf("LEG_TOPIC: %s\n", LEG_TOPIC.c_str());

  JOINT_ANG_N = fsSettings["joint_angle_n"];
  JOINT_VEL_N = fsSettings["joint_vel_n"];
  FOOT_GYR_N = fsSettings["foot_gyro_n"];
  FOOT_GYR_W = fsSettings["foot_gyro_w"];
  FOOT_VEL_W = fsSettings["foot_vel_w"];
  RHO_W = fsSettings["rho_w"];
  ACC_N = fsSettings["acc_n"];
  ACC_N_Z = fsSettings["acc_n_z"];
  ACC_W = fsSettings["acc_w"];
  GYR_N = fsSettings["gyr_n"];
  GYR_W = fsSettings["gyr_w"];
  // print all above parameters
  printf("JOINT_ANG_N: %f\n", JOINT_ANG_N);
  printf("JOINT_VEL_N: %f\n", JOINT_VEL_N);
  printf("FOOT_GYR_N: %f\n", FOOT_GYR_N);
  printf("FOOT_GYR_W: %f\n", FOOT_GYR_W);
  printf("FOOT_VEL_W: %f\n", FOOT_VEL_W);
  printf("RHO_W: %f\n", RHO_W);
  printf("ACC_N: %f\n", ACC_N);
  printf("ACC_W: %f\n", ACC_W);
  printf("GYR_N: %f\n", GYR_N);
  printf("GYR_W: %f\n", GYR_W);

  G.z() = fsSettings["g_norm"];

  SOLVER_TIME = fsSettings["max_solver_time"];
  NUM_ITERATIONS = fsSettings["max_num_iterations"];
  MIN_PARALLAX = fsSettings["keyframe_parallax"];
  MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

  VILO_FUSION_TYPE = fsSettings["vilo_fusion_type"];
  KF_TYPE = fsSettings["kf_type"];

  fsSettings["output_path"] >> OUTPUT_FOLDER;

  std::string kf_type_name = "mipo";
  if (KF_TYPE == 0) {
    kf_type_name = "mipo";
  } else {
    kf_type_name = "sipo";
  }

  ESTIMATE_KINEMATIC = fsSettings["estimate_kinematic"];

  std::string vilo_run_name = "vilo";
  if (VILO_FUSION_TYPE == 0) {
    vilo_run_name = "vio";
  } else if (VILO_FUSION_TYPE == 1) {
    if (KF_TYPE == 0) {
      vilo_run_name = "vilo-m";
    } else {
      vilo_run_name = "vilo-s";
    }
  } else if (VILO_FUSION_TYPE == 2) {
    if (KF_TYPE == 0) {
      if (ESTIMATE_KINEMATIC == 0) {
        vilo_run_name = "vilo-tm-n";
      } else {
        vilo_run_name = "vilo-tm-y";
      }
    } else {
      throw std::runtime_error("not implemented");
    }
  }
  VILO_RESULT_PATH = OUTPUT_FOLDER + "/" + vilo_run_name + "-" + DATASET_NAME + ".csv";
  PO_RESULT_PATH = OUTPUT_FOLDER + "/" + kf_type_name + "-" + DATASET_NAME + ".csv";
  GT_RESULT_PATH = OUTPUT_FOLDER + "/gt" + "-" + DATASET_NAME + ".csv";
  std::cout << "result path " << VILO_RESULT_PATH << std::endl;
  std::ofstream fout_V(VILO_RESULT_PATH, std::ios::out);
  fout_V.close();

  if (VILO_FUSION_TYPE == 0) {
    std::ofstream fout_P(PO_RESULT_PATH, std::ios::out);
    fout_P.close();
  }
  std::ofstream fout_G(GT_RESULT_PATH, std::ios::out);
  fout_G.close();

  ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
  if (ESTIMATE_EXTRINSIC == 0) {
    ROS_WARN(" fix extrinsic param ");
  } else {
    ROS_WARN(" Optimize extrinsic param around initial guess!");
    EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
  }

  cv::Mat cv_T;
  fsSettings["body_T_cam0"] >> cv_T;
  Eigen::Matrix4d T;
  cv::cv2eigen(cv_T, T);
  RIC.push_back(T.block<3, 3>(0, 0));
  TIC.push_back(T.block<3, 1>(0, 3));

  NUM_OF_CAM = fsSettings["num_of_cam"];
  printf("camera number %d\n", NUM_OF_CAM);

  if (NUM_OF_CAM != 2) {
    printf("num_of_cam should be 2\n");
    assert(0);
  }

  int pn = config_file.find_last_of('/');
  std::string configPath = config_file.substr(0, pn);

  std::string cam0Calib;
  fsSettings["cam0_calib"] >> cam0Calib;
  std::string cam0Path = configPath + "/" + cam0Calib;
  CAM_NAMES.push_back(cam0Path);

  if (NUM_OF_CAM == 2) {
    STEREO = 1;
    std::string cam1Calib;
    fsSettings["cam1_calib"] >> cam1Calib;
    std::string cam1Path = configPath + "/" + cam1Calib;
    // printf("%s cam1 path\n", cam1Path.c_str() );
    CAM_NAMES.push_back(cam1Path);

    cv::Mat cv_T;
    fsSettings["body_T_cam1"] >> cv_T;
    Eigen::Matrix4d T;
    cv::cv2eigen(cv_T, T);
    RIC.push_back(T.block<3, 3>(0, 0));
    TIC.push_back(T.block<3, 1>(0, 3));
  }

  INIT_DEPTH = 5.0;
  BIAS_ACC_THRESHOLD = 0.1;
  BIAS_GYR_THRESHOLD = 0.1;

  TD = fsSettings["td"];
  ESTIMATE_TD = fsSettings["estimate_td"];
  if (ESTIMATE_TD)
    ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
  else
    ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

  ROW = fsSettings["image_height"];
  COL = fsSettings["image_width"];
  ROS_INFO("ROW: %d COL: %d ", ROW, COL);

  fsSettings.release();
}
