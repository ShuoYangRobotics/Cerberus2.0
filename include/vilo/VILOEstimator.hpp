#pragma once
#include "featureTracker/feature_manager.h"
#include "featureTracker/feature_tracker.h"

class VILOEstimator {
 public:
  VILOEstimator();
  ~VILOEstimator();

 private:
  std::unique_ptr<FeatureTracker> feature_tracker_;
  std::unique_ptr<FeatureManager> feature_manager_;

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