#include "utils/visualization.h"
#include "vilo/VILOEstimator.hpp"

VILOEstimator::VILOEstimator() {
  feature_manager_ = std::make_unique<FeatureManager>(Rs);
  feature_tracker_ = std::make_unique<FeatureTracker>();
  setParameter();
}

VILOEstimator::~VILOEstimator() {}

void VILOEstimator::setParameter() {
  mProcess.lock();
  for (int i = 0; i < NUM_OF_CAM; i++) {
    tic[i] = TIC[i];
    ric[i] = RIC[i];
    cout << " exitrinsic cam " << i << endl
         << ric[i] << endl
         << tic[i].transpose() << endl;
  }
  feature_manager_->setRic(ric);
  // ProjectionTwoFrameOneCamFactor::sqrt_info =
  //     FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  // ProjectionTwoFrameTwoCamFactor::sqrt_info =
  //     FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  // ProjectionOneFrameTwoCamFactor::sqrt_info =
  //     FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  td = TD;
  g = G;
  cout << "set g " << g.transpose() << endl;
  feature_tracker_->readIntrinsicParameter(CAM_NAMES);

  processThread = std::thread(&VILOEstimator::processMeasurements, this);
  mProcess.unlock();
}

void VILOEstimator::inputImage(double t, const cv::Mat &_img,
                               const cv::Mat &_img1) {
  inputImageCnt++;
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
  TicToc featureTrackerTime;

  if (_img1.empty())
    featureFrame = feature_tracker_->trackImage(t, _img);
  else
    featureFrame = feature_tracker_->trackImage(t, _img, _img1);
  // printf("featureTracker time: %f\n", featureTrackerTime.toc());

  if (SHOW_TRACK) {
    cv::Mat imgTrack = feature_tracker_->getTrackImage();
    Utils::pubTrackImage(imgTrack, t);
  }

  // TODO: understand why MULTIPLE_THREAD has this if statement
  // if (inputImageCnt % 2 == 0) {
  // mBuf.lock();
  // featureBuf.push(make_pair(t, featureFrame));
  // mBuf.unlock();
  // }
}

void VILOEstimator::processMeasurements() {}