#pragma once
#include <Eigen/Dense>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/opencv.hpp>
#include <thread>

#include "featureTracker/feature_manager.h"
#include "featureTracker/feature_tracker.h"

class VILOEstimator {
public:
  VILOEstimator();
  ~VILOEstimator();

  void setParameter();

  void inputImage(double t, const cv::Mat &_img,
                  const cv::Mat &_img1 = cv::Mat());

  // main function
  void processMeasurements();

private:
  std::unique_ptr<FeatureTracker> feature_tracker_;
  std::unique_ptr<FeatureManager> feature_manager_;

  std::thread processThread;

  // IMU, visual buffers and their mutexes
  std::mutex mProcess;
  std::mutex mBuf;
  std::mutex mPropagate;
  queue<pair<double, Eigen::Vector3d>> accBuf;
  queue<pair<double, Eigen::Vector3d>> gyrBuf;
  queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>>>
      featureBuf;
  int inputImageCnt;

  Vector3d g;
  // the actual solved results, Ps Vs Rs are the pose of the imu link
  Matrix3d ric[2];
  Vector3d tic[2];
  Vector3d Ps[(WINDOW_SIZE + 1)];
  Vector3d Vs[(WINDOW_SIZE + 1)];
  Matrix3d Rs[(WINDOW_SIZE + 1)];
  Vector3d Bas[(WINDOW_SIZE + 1)];
  Vector3d Bgs[(WINDOW_SIZE + 1)];
  double td;

  Matrix3d back_R0, last_R, last_R0;
  Vector3d back_P0, last_P, last_P0;
  double Headers[(WINDOW_SIZE + 1)];
};