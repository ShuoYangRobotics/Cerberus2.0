#pragma once
#include <opencv2/imgproc/imgproc_c.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <thread>

#include "featureTracker/feature_manager.h"
#include "featureTracker/feature_tracker.h"
#include "utils/LOTightUtils.hpp"
#include "utils/vins_utility.h"
#include "vilo/initial_alignment.h"

#include "factor/imu_factor.hpp"
#include "factor/integration_base.hpp"

#include "factor/lo_factor.hpp"
#include "factor/lo_intergration_base.hpp"

#include "factor/lo_constant_factor.hpp"

#include "factor/lo_tight_factor.hpp"
#include "factor/lo_tight_integration_base.hpp"

#include "factor/marginalization_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projectionOneFrameTwoCamFactor.h"
#include "factor/projectionTwoFrameOneCamFactor.h"
#include "factor/projectionTwoFrameTwoCamFactor.h"

#define VS_OUTSIZE 17

class VILOEstimator {
 public:
  VILOEstimator();
  ~VILOEstimator();

  void setParameter();
  void reset();

  void inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>& featureFrame);
  void inputImage(double t, const cv::Mat& _img, const cv::Mat& _img1 = cv::Mat());
  void inputBodyIMU(double t, const Vector3d& linearAcceleration, const Vector3d& angularVelocity);

  // this function is used for VILO_FUSION_TYPE == 1
  void inputLOVel(double t, const Vector3d& linearVelocity, const Matrix3d& linearVelocityCov);

  // this function is used for VILO_FUSION_TYPE == 2
  void inputBodyIMULeg(double t, const Vector3d& bodyLinearAcceleration, const Vector3d& bodyAngularVelocity,
                       const Eigen::Matrix<double, 12, 1>& footAngularVelocity, const Eigen::Matrix<double, NUM_DOF, 1>& jointAngles,
                       const Eigen::Matrix<double, NUM_DOF, 1>& jointVelocities, const Eigen::Matrix<double, NUM_LEG, 1>& contactFlags);

  // output latest state
  Eigen::Matrix<double, VS_OUTSIZE, 1> outputState() const;
  bool isRunning() const { return (solver_flag == NON_LINEAR) && (frame_count == WINDOW_SIZE); }

  // main function
  void processMeasurements();

 private:
  enum SolverFlag { INITIAL, NON_LINEAR };

  enum MarginalizationFlag { MARGIN_OLD = 0, MARGIN_SECOND_NEW = 1 };

  std::unique_ptr<FeatureTracker> feature_tracker_;
  std::unique_ptr<FeatureManager> feature_manager_;

  bool initThreadFlag;
  std::thread processThread;  // thread that triggers the sliding window solve

  // IMU, visual input buffers and their mutexes
  std::mutex mProcess;
  std::mutex mBuf;
  std::mutex mPropagate;
  queue<pair<double, Eigen::Vector3d>> accBuf;
  queue<pair<double, Eigen::Vector3d>> gyrBuf;
  queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>>> featureBuf;
  int inputImageCnt;

  // leg odometry input buffers
  queue<pair<double, Eigen::Vector3d>> loBuf;
  queue<pair<double, Eigen::Matrix3d>> loCovBuf;

  // tightly coupled LO VILO_FUSION_TYPE == 2
  std::shared_ptr<LOTightUtils> lo_tight_utils_[NUM_LEG];
  queue<Eigen::Vector3d> footGyrBuf[NUM_LEG];
  queue<Eigen::Vector3d> jointAngBuf[NUM_LEG];
  queue<Eigen::Vector3d> jointVelBuf[NUM_LEG];
  queue<double> contactFlagBuf[NUM_LEG];

  // the two adjacent camera frame time
  double prevTime, curTime;

  bool openExEstimation;  // start to estimate extrinsic parameters or not

  Vector3d g;  // gravity vector

  // the actual solved results, Ps Vs Rs are the pose of the imu link
  Matrix3d ric[2];
  Vector3d tic[2];
  Vector3d Ps[(WINDOW_SIZE + 1)];
  Vector3d Vs[(WINDOW_SIZE + 1)];
  Matrix3d Rs[(WINDOW_SIZE + 1)];
  Vector3d Bas[(WINDOW_SIZE + 1)];
  Vector3d Bgs[(WINDOW_SIZE + 1)];
  double td;
  // actual solved results, with leg related VILO_FUSION_TYPE == 2
  Vector3d Bfs[(WINDOW_SIZE + 1)][NUM_LEG];
  Vector3d Bvs[(WINDOW_SIZE + 1)][NUM_LEG];
  Vec_rho Rhos[(WINDOW_SIZE + 1)][NUM_LEG];

  // sliding window related
  int frame_count;  // which frame is in the current sliding window

  // counters for debugging
  int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

  // per frame sensor data buffers
  vector<double> dt_buf[(WINDOW_SIZE + 1)];                     // IMU dt
  vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];  // IMU acc
  vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];     // IMU gyro

  vector<double> lo_dt_buf[(WINDOW_SIZE + 1)];              // leg odometry dt
  vector<Vector3d> lo_velocity_buf[(WINDOW_SIZE + 1)];      // leg odometry velocity
  vector<Matrix3d> lo_velocity_cov_buf[(WINDOW_SIZE + 1)];  // leg odometry velocity cov

  vector<double> tight_lo_dt_buf[(WINDOW_SIZE + 1)][NUM_LEG];         // tightly leg oodmetry
  vector<Vector3d> tight_lo_bodyGyr_buf[(WINDOW_SIZE + 1)][NUM_LEG];  // tightly leg oodmetry
  vector<Vector3d> tight_lo_footGyr_buf[(WINDOW_SIZE + 1)][NUM_LEG];  // tightly leg oodmetry
  vector<Vector3d> tight_lo_jang_buf[(WINDOW_SIZE + 1)][NUM_LEG];     // tightly leg oodmetry
  vector<Vector3d> tight_lo_jvel_buf[(WINDOW_SIZE + 1)][NUM_LEG];     // tightly leg oodmetry

  // failure detection related
  bool failure_occur;

  // process imu
  bool first_imu;
  Vector3d acc_0, gyr_0;  // save previous imu data
  IntegrationBase* pre_integrations[(WINDOW_SIZE + 1)] = {nullptr};
  // process LO vel, loosely couple (lo)
  bool first_lo;
  Vector3d lo_vel_0;  // save previous leg odometry data
  Matrix3d lo_vel_cov_0;
  LOIntegrationBase* lo_pre_integrations[(WINDOW_SIZE + 1)] = {nullptr};

  // process LO vel, tightly couple (tlo)
  bool first_tight_lo[NUM_LEG];
  Vector3d tight_lo_body_gyr_0[NUM_LEG];
  Vector3d tight_lo_foot_gyr_0[NUM_LEG];
  Vector3d tight_lo_joint_ang_0[NUM_LEG];
  Vector3d tight_lo_joint_vel_0[NUM_LEG];
  LOTightIntegrationBase* tlo_pre_integration[(WINDOW_SIZE + 1)][NUM_LEG] = {nullptr};
  bool tlo_all_in_contact[(WINDOW_SIZE + 1)][NUM_LEG] = {
      true};  // this flag indicates with a correspinding tlo_pre_integration term can be used in a tight factor or not. If not (because
              // the contact flag during this period is not all 1), then we add LOConstantFactor instead of LOTightFactor

  bool initFirstPoseFlag;

  Matrix3d back_R0, last_R, last_R0;
  Vector3d back_P0, last_P, last_P0;
  double Headers[(WINDOW_SIZE + 1)];

  // initialization related
  map<double, ImageFrame> all_image_frame;
  IntegrationBase* tmp_pre_integration = nullptr;

  // control solver
  SolverFlag solver_flag;
  MarginalizationFlag marginalization_flag;
  MarginalizationInfo* last_marginalization_info = nullptr;
  vector<double*> last_marginalization_parameter_blocks;

  // ceres solver variable
  double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
  double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
  double para_Feature[NUM_OF_F][SIZE_FEATURE];
  double para_Ex_Pose[2][SIZE_POSE];
  double para_Retrive_Pose[SIZE_POSE];
  double para_Td[1][1];
  double para_Tr[1][1];

  double para_FootBias[WINDOW_SIZE + 1][NUM_LEG][SIZE_FOOTBIAS];

  // extract the latest state
  double latest_time;
  Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
  Eigen::Quaterniond latest_Q;

  // private functions

  bool getBodyIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>>& accVector,
                          vector<pair<double, Eigen::Vector3d>>& gyrVector);
  bool getBodyIMULegInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>>& bodyAccVector,
                             vector<pair<double, Eigen::Vector3d>>& bodyGyrVector, vector<Eigen::Vector3d> footGyrVector[],
                             vector<Eigen::Vector3d> jointAngVector[], vector<Eigen::Vector3d> jointVelVector[], double contactDecision[]);
  bool BodyIMUAvailable(double t);

  bool getLoVelInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>>& loVelVector,
                        vector<pair<double, Eigen::Matrix3d>>& loCovVector);
  bool loVelAvailable(double t);

  void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>>& accVector);
  void processIMU(double t, double dt, const Vector3d& linear_acceleration, const Vector3d& angular_velocity);

  // this is for VILO_FUSION_TYPE == 1
  void processLegOdom(double t, double dt, const Eigen::Vector3d& loVel, const Eigen::Matrix3d& loCov);

  // this is for VILO_FUSION_TYPE == 2
  void processIMULegOdom(int leg_id, double t, double dt, const Vector3d& bodyAngularVelocity, const Vector3d& footAngularVelocity,
                         const Vector3d& jointAngles, const Vector3d& jointVelocities);

  void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>& image, const double header);

  // convert between Eigen and Ceres
  void vector2double();
  void double2vector();

  bool failureDetection();

  // main optimization functions
  void slideWindow();
  void slideWindowNew();
  void slideWindowOld();
  void optimization();

  // get latest state
  void updateLatestStates();

  // feature tracking and prediction helper functions
  void getPoseInWorldFrame(Eigen::Matrix4d& T);
  void getPoseInWorldFrame(int index, Eigen::Matrix4d& T);
  void predictPtsInNextFrame();
  void outliersRejection(set<int>& removeIndex);
  double reprojectionError(Matrix3d& Ri, Vector3d& Pi, Matrix3d& rici, Vector3d& tici, Matrix3d& Rj, Vector3d& Pj, Matrix3d& ricj,
                           Vector3d& ticj, double depth, Vector3d& uvi, Vector3d& uvj);
};