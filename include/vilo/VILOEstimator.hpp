#pragma once
#include <Eigen/Dense>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/opencv.hpp>
#include <thread>

#include "featureTracker/feature_manager.h"
#include "featureTracker/feature_tracker.h"
#include "utils/vins_utility.h"
#include "vilo/initial_alignment.h"

#include "factor/imu_factor.hpp"
#include "factor/integration_base.hpp"
#include "factor/marginalization_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projectionOneFrameTwoCamFactor.h"
#include "factor/projectionTwoFrameOneCamFactor.h"
#include "factor/projectionTwoFrameTwoCamFactor.h"

class VILOEstimator {
public:
  VILOEstimator();
  ~VILOEstimator();

  void setParameter();
  void reset();

  void
  inputFeature(double t,
               const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>
                   &featureFrame);
  void inputImage(double t, const cv::Mat &_img,
                  const cv::Mat &_img1 = cv::Mat());
  void inputBodyIMU(double t, const Vector3d &linearAcceleration,
                    const Vector3d &angularVelocity);
  void inputLOVel(double t, const Vector3d &linearVelocity);

  // output latest state
  Eigen::VectorXd outputState() const;

  // main function
  void processMeasurements();

private:
  enum SolverFlag { INITIAL, NON_LINEAR };

  enum MarginalizationFlag { MARGIN_OLD = 0, MARGIN_SECOND_NEW = 1 };

  std::unique_ptr<FeatureTracker> feature_tracker_;
  std::unique_ptr<FeatureManager> feature_manager_;

  bool initThreadFlag;
  std::thread processThread; // thread that triggers the sliding window solve

  // IMU, visual input buffers and their mutexes
  std::mutex mProcess;
  std::mutex mBuf;
  std::mutex mPropagate;
  queue<pair<double, Eigen::Vector3d>> accBuf;
  queue<pair<double, Eigen::Vector3d>> gyrBuf;
  queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>>>
      featureBuf;
  int inputImageCnt;

  // the two adjacent camera frame time
  double prevTime, curTime;

  bool openExEstimation; // start to estimate extrinsic parameters or not

  Vector3d g; // gravity vector

  // the actual solved results, Ps Vs Rs are the pose of the imu link
  Matrix3d ric[2];
  Vector3d tic[2];
  Vector3d Ps[(WINDOW_SIZE + 1)];
  Vector3d Vs[(WINDOW_SIZE + 1)];
  Matrix3d Rs[(WINDOW_SIZE + 1)];
  Vector3d Bas[(WINDOW_SIZE + 1)];
  Vector3d Bgs[(WINDOW_SIZE + 1)];
  double td;

  // sliding window related
  int frame_count; // which frame is in the current sliding window

  // counters for debugging
  int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

  // per frame sensor data buffers
  vector<double> dt_buf[(WINDOW_SIZE + 1)];                    // IMU dt
  vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)]; // IMU acc
  vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];    // IMU gyro
  vector<double> lo_dt_buf[(WINDOW_SIZE + 1)];         // leg odometry dt
  vector<Vector3d> lo_velocity_buf[(WINDOW_SIZE + 1)]; // leg odometry velocity

  // failure detection related
  bool failure_occur;

  // process imu
  bool first_imu;
  Vector3d acc_0, gyr_0; // save previous imu data
  IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)] = {nullptr};

  bool initFirstPoseFlag;

  Matrix3d back_R0, last_R, last_R0;
  Vector3d back_P0, last_P, last_P0;
  double Headers[(WINDOW_SIZE + 1)];

  // initialization related
  map<double, ImageFrame> all_image_frame;
  IntegrationBase *tmp_pre_integration = nullptr;

  // control solver
  SolverFlag solver_flag;
  MarginalizationFlag marginalization_flag;
  MarginalizationInfo *last_marginalization_info = nullptr;
  vector<double *> last_marginalization_parameter_blocks;

  // ceres solver variable
  double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
  double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
  double para_Feature[NUM_OF_F][SIZE_FEATURE];
  double para_Ex_Pose[2][SIZE_POSE];
  double para_Retrive_Pose[SIZE_POSE];
  double para_Td[1][1];
  double para_Tr[1][1];

  // extract the latest state
  double latest_time;
  Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0,
      latest_gyr_0;
  Eigen::Quaterniond latest_Q;

  // private functions

  bool getBodyIMUInterval(double t0, double t1,
                          vector<pair<double, Eigen::Vector3d>> &accVector,
                          vector<pair<double, Eigen::Vector3d>> &gyrVector);
  bool BodyIMUAvailable(double t);

  void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);
  void processIMU(double t, double dt, const Vector3d &linear_acceleration,
                  const Vector3d &angular_velocity);

  void processImage(
      const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
      const double header);

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
  void getPoseInWorldFrame(Eigen::Matrix4d &T);
  void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
  void predictPtsInNextFrame();
  void outliersRejection(set<int> &removeIndex);
  double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici,
                           Vector3d &tici, Matrix3d &Rj, Vector3d &Pj,
                           Matrix3d &ricj, Vector3d &ticj, double depth,
                           Vector3d &uvi, Vector3d &uvj);
};