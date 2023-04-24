#include "vilo/VILOEstimator.hpp"
#include "utils/vins_utility.h"
#include "utils/visualization.h"

VILOEstimator::VILOEstimator() {
  initThreadFlag = false;
  feature_manager_ = std::make_unique<FeatureManager>(Rs);
  feature_tracker_ = std::make_unique<FeatureTracker>();

  reset();
}

VILOEstimator::~VILOEstimator() {
  processThread.join();
}

void VILOEstimator::setParameter() {
  const std::lock_guard<std::mutex> lock(mProcess);
  for (int i = 0; i < NUM_OF_CAM; i++) {
    tic[i] = TIC[i];
    ric[i] = RIC[i];
    cout << " exitrinsic cam " << i << endl << ric[i] << endl << tic[i].transpose() << endl;
  }
  feature_manager_->setRic(ric);
  ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  td = TD;
  g = G;
  cout << "set g " << g.transpose() << endl;
  feature_tracker_->readIntrinsicParameter(CAM_NAMES);

  // this ensures thread is only started once even setParameter is called
  // multiple times
  if (initThreadFlag == false) {
    processThread = std::thread(&VILOEstimator::processMeasurements, this);
    initThreadFlag = true;
  }
}

void VILOEstimator::reset() {
  const std::lock_guard<std::mutex> lock(mProcess);

  while (!accBuf.empty()) accBuf.pop();
  while (!gyrBuf.empty()) gyrBuf.pop();
  while (!featureBuf.empty()) featureBuf.pop();

  if (VILO_FUSION_TYPE == 2) {
    for (int j = 0; j < NUM_LEG; j++) {
      while (!footGyrBuf[j].empty()) footGyrBuf[j].pop();
      while (!jointAngBuf[j].empty()) jointAngBuf[j].pop();
      while (!jointVelBuf[j].empty()) jointVelBuf[j].pop();
      while (!contactFlagBuf[j].empty()) contactFlagBuf[j].pop();
    }
  }

  if (VILO_FUSION_TYPE == 1) {
    while (!loBuf.empty()) loBuf.pop();
    while (!loCovBuf.empty()) loCovBuf.pop();
  }

  // frame_count rest, most important
  frame_count = 0;

  // reset camera frame time
  prevTime = -1;
  curTime = 0;

  // do not do extrinsic estimation at the beginning
  openExEstimation = 0;
  inputImageCnt = 0;

  // reset the sliding window solver
  solver_flag = INITIAL;

  for (int i = 0; i < WINDOW_SIZE + 1; i++) {
    Rs[i].setIdentity();
    Ps[i].setZero();
    Vs[i].setZero();
    Bas[i].setZero();
    Bgs[i].setZero();

    for (int j = 0; j < NUM_LEG; j++) {
      Bfs[i][j].setZero();
      Bvs[i][j].setZero();
      Rhos[i][j].setZero();
    }

    dt_buf[i].clear();
    linear_acceleration_buf[i].clear();
    angular_velocity_buf[i].clear();

    if (pre_integrations[i] != nullptr) {
      delete pre_integrations[i];
    }
    pre_integrations[i] = nullptr;

    if (VILO_FUSION_TYPE == 1) {
      lo_velocity_buf[i].clear();
      lo_velocity_cov_buf[i].clear();
      lo_dt_buf[i].clear();
      if (lo_pre_integrations[i] != nullptr) {
        delete lo_pre_integrations[i];
      }
      lo_pre_integrations[i] = nullptr;
    }

    if (VILO_FUSION_TYPE == 2) {
      for (int j = 0; j < NUM_LEG; j++) {
        tight_lo_dt_buf[i][j].clear();
        tight_lo_bodyGyr_buf[i][j].clear();
        tight_lo_footGyr_buf[i][j].clear();
        tight_lo_jang_buf[i][j].clear();
        tight_lo_jvel_buf[i][j].clear();

        if (tlo_pre_integration[i][j] != nullptr) {
          delete tlo_pre_integration[i][j];
        }
        tlo_pre_integration[i][j] = nullptr;
        tlo_all_in_contact[i][j] = true;
      }
    }
  }

  for (int i = 0; i < NUM_OF_CAM; i++) {
    tic[i] = TIC[i];
    ric[i] = RIC[i];
  }

  first_imu = false;
  initFirstPoseFlag = false;

  if (tmp_pre_integration != nullptr) delete tmp_pre_integration;
  tmp_pre_integration = nullptr;

  if (last_marginalization_info != nullptr) delete last_marginalization_info;
  last_marginalization_info = nullptr;
  last_marginalization_parameter_blocks.clear();

  // two unused counters
  sum_of_back = 0;
  sum_of_front = 0;

  failure_occur = 0;

  // clear the feature manager
  feature_manager_->clearState();
  all_image_frame.clear();
}

void VILOEstimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>& featureFrame) {
  const std::lock_guard<std::mutex> lock(mBuf);
  featureBuf.push(make_pair(t, featureFrame));
}

void VILOEstimator::inputImage(double t, const cv::Mat& _img, const cv::Mat& _img1) {
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
  const std::lock_guard<std::mutex> lock(mBuf);
  featureBuf.push(make_pair(t, featureFrame));
  // }
}

void VILOEstimator::inputBodyIMU(double t, const Vector3d& linearAcceleration, const Vector3d& angularVelocity) {
  const std::lock_guard<std::mutex> lock(mBuf);
  accBuf.push(make_pair(t, linearAcceleration));
  gyrBuf.push(make_pair(t, angularVelocity));
  // printf("input imu with time %f \n", t);
}

// this function is used for VILO_FUSION_TYPE == 1, it is used along with inputBodyIMU
void VILOEstimator::inputLOVel(double t, const Vector3d& linearVelocity, const Matrix3d& linearVelocityCov) {
  const std::lock_guard<std::mutex> lock(mBuf);
  loBuf.push(make_pair(t, linearVelocity));
  loCovBuf.push(make_pair(t, linearVelocityCov));
}

// this function is used for VILO_FUSION_TYPE == 2, it replaces inputBodyIMU
void VILOEstimator::inputBodyIMULeg(double t, const Vector3d& bodyLinearAcceleration, const Vector3d& bodyAngularVelocity,
                                    const Eigen::Matrix<double, 12, 1>& footAngularVelocity,
                                    const Eigen::Matrix<double, NUM_DOF, 1>& jointAngles,
                                    const Eigen::Matrix<double, NUM_DOF, 1>& jointVelocities,
                                    const Eigen::Matrix<double, NUM_LEG, 1>& contactFlags) {
  const std::lock_guard<std::mutex> lock(mBuf);
  accBuf.push(make_pair(t, bodyLinearAcceleration));
  gyrBuf.push(make_pair(t, bodyAngularVelocity));

  for (int j = 0; j < NUM_LEG; j++) {
    footGyrBuf[j].push(footAngularVelocity.segment<3>(j * 3));
    jointAngBuf[j].push(jointAngles.segment<3>(j * 3));
    jointVelBuf[j].push(jointVelocities.segment<3>(j * 3));
    contactFlagBuf[j].push(contactFlags[j]);
  }
}

bool VILOEstimator::getBodyIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>>& accVector,
                                       vector<pair<double, Eigen::Vector3d>>& gyrVector) {
  if (accBuf.empty()) {
    printf("not receive imu\n");
    return false;
  }
  // printf("get imu from %f %f\n", t0, t1);
  // printf("imu fornt time %f   imu end time %f\n", accBuf.front().first,
  // accBuf.back().first);
  if (t1 <= accBuf.back().first) {
    while (accBuf.front().first <= t0) {
      accBuf.pop();
      gyrBuf.pop();
    }
    while (accBuf.front().first < t1) {
      accVector.push_back(accBuf.front());
      accBuf.pop();
      gyrVector.push_back(gyrBuf.front());
      gyrBuf.pop();
    }
    accVector.push_back(accBuf.front());
    gyrVector.push_back(gyrBuf.front());
  } else {
    printf("wait for imu\n");
    return false;
  }
  return true;
}

// this replaces getBodyIMUInterval when VILO_FUSION_TYPE == 2
// contactDecision is very important, it shows whether during t0 and t1
bool VILOEstimator::getBodyIMULegInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>>& bodyAccVector,
                                          vector<pair<double, Eigen::Vector3d>>& bodyGyrVector, vector<Eigen::Vector3d> footGyrVector[],
                                          vector<Eigen::Vector3d> jointAngVector[], vector<Eigen::Vector3d> jointVelVector[],
                                          double contactDecision[]) {
  if (accBuf.empty()) {
    printf("not receive imu and leg\n");
    return false;
  }
  // printf("get imu from %f %f\n", t0, t1);
  // printf("imu fornt time %f   imu end time %f\n", accBuf.front().first,
  // accBuf.back().first);
  if (t1 <= accBuf.back().first) {
    while (accBuf.front().first <= t0) {
      accBuf.pop();
      gyrBuf.pop();
      for (int j = 0; j < NUM_LEG; j++) {
        footGyrBuf[j].pop();
        jointAngBuf[j].pop();
        jointVelBuf[j].pop();
        contactFlagBuf[j].pop();
      }
    }
    // the actual important place, first set all contactDecision[] to be true
    for (int j = 0; j < NUM_LEG; j++) {
      contactDecision[j] = true;
    }
    while (accBuf.front().first < t1) {
      bodyAccVector.push_back(accBuf.front());
      accBuf.pop();

      bodyGyrVector.push_back(gyrBuf.front());
      gyrBuf.pop();

      for (int j = 0; j < NUM_LEG; j++) {
        footGyrVector[j].push_back(footGyrBuf[j].front());
        footGyrBuf[j].pop();
        jointAngVector[j].push_back(jointAngBuf[j].front());
        jointAngBuf[j].pop();
        jointVelVector[j].push_back(jointVelBuf[j].front());
        jointVelBuf[j].pop();

        if (contactFlagBuf[j].front() != 1.0) {
          contactDecision[j] = false;
        }
        contactFlagBuf[j].pop();
      }
    }
    bodyAccVector.push_back(accBuf.front());
    bodyGyrVector.push_back(gyrBuf.front());

    for (int j = 0; j < NUM_LEG; j++) {
      footGyrVector[j].push_back(footGyrBuf[j].front());
      jointAngVector[j].push_back(jointAngBuf[j].front());
      jointVelVector[j].push_back(jointVelBuf[j].front());
      if (contactFlagBuf[j].front() != 1.0) {
        contactDecision[j] = false;
      }
    }

  } else {
    printf("wait for imu and leg\n");
    return false;
  }
  return true;
}

bool VILOEstimator::getLoVelInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>>& loVelVector,
                                     vector<pair<double, Eigen::Matrix3d>>& loCovVector) {
  if (loBuf.empty()) {
    printf("not receive loVel\n");
    return false;
  }
  if (t1 <= loBuf.back().first) {
    while (loBuf.front().first <= t0) {
      loBuf.pop();
    }
    while (loBuf.front().first < t1) {
      loVelVector.push_back(loBuf.front());
      loBuf.pop();
    }
    loVelVector.push_back(loBuf.front());
    // do the same thing of loCovBuf using loCovVector
    while (loCovBuf.front().first <= t0) {
      loCovBuf.pop();
    }
    while (loCovBuf.front().first < t1) {
      loCovVector.push_back(loCovBuf.front());
      loCovBuf.pop();
    }
    loCovVector.push_back(loCovBuf.front());

  } else {
    printf("wait for loVel\n");
    return false;
  }
  return true;
}

bool VILOEstimator::BodyIMUAvailable(double t) {
  if (!accBuf.empty() && t <= accBuf.back().first)
    return true;
  else
    return false;
}

bool VILOEstimator::loVelAvailable(double t) {
  if (!loBuf.empty() && t <= loBuf.back().first)
    return true;
  else
    return false;
}

void VILOEstimator::processMeasurements() {
  std::chrono::milliseconds dura(2);
  while (ros::ok()) {
    pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> feature;
    vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
    vector<Eigen::Vector3d> footGyrVector[NUM_LEG], jointAngVector[NUM_LEG], jointVelVector[NUM_LEG];
    double contactDecision[NUM_LEG];

    vector<pair<double, Eigen::Vector3d>> loVelVector;
    vector<pair<double, Eigen::Matrix3d>> loCovVector;

    if (!featureBuf.empty()) {
      std::cout << "process measurments" << std::endl;
      feature = featureBuf.front();
      curTime = feature.first + td;
      while (ros::ok()) {
        if (BodyIMUAvailable(feature.first + td))
          break;
        else {
          // printf("wait for imu ... \n");
          std::this_thread::sleep_for(dura);
        }
      }
      mBuf.lock();

      if (VILO_FUSION_TYPE == 2) {
        getBodyIMULegInterval(prevTime, curTime, accVector, gyrVector, footGyrVector, jointAngVector, jointVelVector, contactDecision);
        // now contactDecision[j] contains flag indicating between prevTime and curTime whether leg j is in contact
        // TODO: check this
        for (int j = 0; j < NUM_LEG; j++) {
          tlo_all_in_contact[frame_count][j] = contactDecision[j];
        }
      } else {
        getBodyIMUInterval(prevTime, curTime, accVector, gyrVector);
      }
      featureBuf.pop();

      mBuf.unlock();

      if (!initFirstPoseFlag) {
        initFirstIMUPose(accVector);
      }
      std::cout << "prevTime " << setprecision(15) << prevTime << std::endl;
      std::cout << "curTime " << setprecision(15) << curTime << std::endl;

      for (size_t i = 0; i < accVector.size(); i++) {
        double dt;
        if (i == 0)
          dt = accVector[i].first - prevTime;
        else if (i == accVector.size() - 1)
          dt = curTime - accVector[i - 1].first;
        else
          dt = accVector[i].first - accVector[i - 1].first;
        processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);

        // do contact preintegration for leg in contact
        if (VILO_FUSION_TYPE == 2) {
          for (int j = 0; j < NUM_LEG; j++) {
            if (contactDecision[j]) {
              processIMULegOdom(j, accVector[i].first, dt, gyrVector[i].second, footGyrVector[j][i], jointAngVector[j][i],
                                jointVelVector[j][i]);
            }
          }
        }
      }

      // process LO velocity
      if (VILO_FUSION_TYPE == 1) {
        // obtain mBuf lock
        std::lock_guard<std::mutex> lock(mBuf);
        getLoVelInterval(prevTime, curTime, loVelVector, loCovVector);
        for (size_t i = 0; i < loVelVector.size(); i++) {
          double dt;
          if (i == 0)
            dt = loVelVector[i].first - prevTime;
          else if (i == loVelVector.size() - 1)
            dt = curTime - loVelVector[i - 1].first;
          else
            dt = loVelVector[i].first - loVelVector[i - 1].first;
          processLegOdom(loVelVector[i].first, dt, loVelVector[i].second, loCovVector[i].second);
        }
      }

      const std::lock_guard<std::mutex> lock(mProcess);
      processImage(feature.second, feature.first);
      prevTime = curTime;
    }

    std::this_thread::sleep_for(dura);
  }
}

void VILOEstimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>>& accVector) {
  printf("init first imu pose\n");
  initFirstPoseFlag = true;
  // return;
  Eigen::Vector3d averAcc(0, 0, 0);
  int n = (int)accVector.size();
  for (size_t i = 0; i < accVector.size(); i++) {
    averAcc = averAcc + accVector[i].second;
  }
  averAcc = averAcc / n;
  printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
  Matrix3d R0 = Utility::g2R(averAcc);
  double yaw = Utility::R2ypr(R0).x();
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  Rs[0] = R0;
  cout << "init R0 " << endl << Rs[0] << endl;
  // Vs[0] = Vector3d(5, 0, 0);
}

// output latest state
Eigen::Matrix<double, VS_OUTSIZE, 1> VILOEstimator::outputState() const {
  Eigen::Matrix<double, VS_OUTSIZE, 1> state;
  state(0) = latest_time;
  state.segment(1, 3) = latest_P;
  state.segment(4, 4) = Eigen::Quaterniond(latest_Q).coeffs();  // x y z w
  state.segment(8, 3) = latest_V;
  state.segment(11, 3) = latest_Ba;
  state.segment(14, 3) = latest_Bg;

  return state;
}

void VILOEstimator::processIMU(double t, double dt, const Vector3d& linear_acceleration, const Vector3d& angular_velocity) {
  if (!first_imu) {
    first_imu = true;
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
  }

  if (!pre_integrations[frame_count]) {
    pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
  }
  if (frame_count != 0) {
    pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
    // if(solver_flag != NON_LINEAR)
    tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

    dt_buf[frame_count].push_back(dt);
    linear_acceleration_buf[frame_count].push_back(linear_acceleration);
    angular_velocity_buf[frame_count].push_back(angular_velocity);

    // TODO: IMU acc may be noise, so a better solution for initialization may
    // be is to use LO velocity
    int j = frame_count;

    Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
    Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();

    // initialize P and V using IMU integration
    if (VILO_FUSION_TYPE == 0) {
      Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
      Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
      Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
      Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
      Vs[j] += dt * un_acc;
    }
  }
  acc_0 = linear_acceleration;
  gyr_0 = angular_velocity;
}
// this is used in VILO_FUSION_TYPE == 2 together with processIMU
void VILOEstimator::processIMULegOdom(int leg_id, double t, double dt, const Vector3d& bodyAngularVelocity,
                                      const Vector3d& footAngularVelocity, const Vector3d& jointAngles, const Vector3d& jointVelocities) {
  if (!first_tight_lo[leg_id]) {
    first_tight_lo[leg_id] = true;
    tight_lo_body_gyr_0[leg_id] = bodyAngularVelocity;
    tight_lo_foot_gyr_0[leg_id] = footAngularVelocity;
    tight_lo_joint_ang_0[leg_id] = jointAngles;
    tight_lo_joint_vel_0[leg_id] = jointVelocities;
  }

  if (!tlo_pre_integration[frame_count][leg_id]) {
    tlo_pre_integration[frame_count][leg_id] = new LOTightIntegrationBase{leg_id,
                                                                          tight_lo_joint_ang_0[leg_id],
                                                                          tight_lo_joint_vel_0[leg_id],
                                                                          tight_lo_body_gyr_0[leg_id],
                                                                          tight_lo_foot_gyr_0[leg_id],
                                                                          Bgs[frame_count],
                                                                          Bfs[frame_count][leg_id],
                                                                          Bvs[frame_count][leg_id],
                                                                          Rhos[frame_count][leg_id],
                                                                          &lo_tight_utils_[leg_id]};
  }

  if (frame_count != 0) {
    tlo_pre_integration[frame_count][leg_id]->push_back(dt, bodyAngularVelocity, footAngularVelocity, jointAngles, jointVelocities);
    tight_lo_dt_buf[frame_count][leg_id].push_back(dt);
    tight_lo_bodyGyr_buf[frame_count][leg_id].push_back(bodyAngularVelocity);
    tight_lo_footGyr_buf[frame_count][leg_id].push_back(footAngularVelocity);
    tight_lo_jang_buf[frame_count][leg_id].push_back(jointAngles);
    tight_lo_jvel_buf[frame_count][leg_id].push_back(jointVelocities);
  }

  tight_lo_body_gyr_0[leg_id] = bodyAngularVelocity;
  tight_lo_foot_gyr_0[leg_id] = footAngularVelocity;
  tight_lo_joint_ang_0[leg_id] = jointAngles;
  tight_lo_joint_vel_0[leg_id] = jointVelocities;
}

// this is used in VILO_FUSION_TYPE == 1 together with processIMU
void VILOEstimator::processLegOdom(double t, double dt, const Eigen::Vector3d& loVel, const Eigen::Matrix3d& loCov) {
  if (!first_lo) {
    first_lo = true;
    lo_vel_0 = loVel;
    lo_vel_cov_0 = loCov;
  }

  // contact preintegration
  if (!lo_pre_integrations[frame_count]) {
    lo_pre_integrations[frame_count] = new LOIntegrationBase{lo_vel_0, lo_vel_cov_0};
  }

  if (frame_count != 0) {
    lo_pre_integrations[frame_count]->push_back(dt, loVel, loCov);
    lo_dt_buf[frame_count].push_back(dt);
    lo_velocity_buf[frame_count].push_back(loVel);
    lo_velocity_cov_buf[frame_count].push_back(loCov);

    // use LO to provide initial guess for P and V
    int j = frame_count;
    Vector3d average_vel = 0.5 * (lo_vel_0 + loVel);
    Vs[j] = average_vel;  // world frame velocity from LO
    Ps[j] += dt * Vs[j];
  }
  lo_vel_0 = loVel;
  lo_vel_cov_0 = loCov;
  return;
}

void VILOEstimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>& image, const double header) {
  // first determine marginalization flag
  if (feature_manager_->addFeatureCheckParallax(frame_count, image, td)) {
    marginalization_flag = MARGIN_OLD;
    // printf("keyframe\n");
  } else {
    marginalization_flag = MARGIN_SECOND_NEW;
    // printf("non-keyframe\n");
  }

  Headers[frame_count] = header;
  ImageFrame imageframe(image, header);
  imageframe.pre_integration = tmp_pre_integration;
  all_image_frame.insert(make_pair(header, imageframe));
  tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

  if (solver_flag == INITIAL) {  // stereo + IMU initialization
    feature_manager_->initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
    feature_manager_->triangulate(frame_count, Ps, Rs, tic, ric);
    if (frame_count == WINDOW_SIZE) {
      map<double, ImageFrame>::iterator frame_it;
      int i = 0;
      for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++) {
        frame_it->second.R = Rs[i];
        frame_it->second.T = Ps[i];
        i++;
      }
      solveGyroscopeBias(all_image_frame, Bgs);
      for (int i = 0; i <= WINDOW_SIZE; i++) {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
      }
      optimization();
      updateLatestStates();
      solver_flag = NON_LINEAR;
      slideWindow();
      ROS_INFO("Initialization finish!");
    }

    if (frame_count < WINDOW_SIZE) {
      frame_count++;
      int prev_frame = frame_count - 1;
      Ps[frame_count] = Ps[prev_frame];
      Vs[frame_count] = Vs[prev_frame];
      Rs[frame_count] = Rs[prev_frame];
      Bas[frame_count] = Bas[prev_frame];
      Bgs[frame_count] = Bgs[prev_frame];

      // for VILO_FUSION_TYPE == 2
      for (int j = 0; j < NUM_LEG; j++) {
        Bfs[frame_count][j] = Bfs[prev_frame][j];
        Bvs[frame_count][j] = Bvs[prev_frame][j];
        Rhos[frame_count][j] = Rhos[prev_frame][j];
      }
    }
  } else {  // the usual sliding window optimization

    feature_manager_->triangulate(frame_count, Ps, Rs, tic, ric);
    optimization();
    set<int> removeIndex;
    outliersRejection(removeIndex);
    feature_manager_->removeOutlier(removeIndex);

    if (failureDetection()) {
      ROS_WARN("failure detection!");
      failure_occur = 1;
      reset();
      setParameter();
      ROS_WARN("system reboot!");
      return;
    }
    slideWindow();
    feature_manager_->removeFailures();

    last_R = Rs[WINDOW_SIZE];
    last_P = Ps[WINDOW_SIZE];
    last_R0 = Rs[0];
    last_P0 = Ps[0];
    updateLatestStates();
  }
  // std cout solver flag and frame count
  std::cout << "solver_flag: " << solver_flag << std::endl;
  std::cout << "frame_count: " << frame_count << std::endl;
  Eigen::VectorXd x_vilo = outputState();
  Eigen::Vector3d pos = x_vilo.segment<3>(1);
  Eigen::Quaterniond quat(x_vilo(7), x_vilo(4), x_vilo(5), x_vilo(6));
  Eigen::Vector3d vel = x_vilo.segment(8, 3);
  printf("time: %f, t: %f %f %f q: %f %f %f %f \n", ros::Time::now().toSec(), pos(0), pos(1), pos(2), quat.w(), quat.x(), quat.y(),
         quat.z());
}

void VILOEstimator::vector2double() {
  for (int i = 0; i <= WINDOW_SIZE; i++) {
    para_Pose[i][0] = Ps[i].x();
    para_Pose[i][1] = Ps[i].y();
    para_Pose[i][2] = Ps[i].z();
    Quaterniond q{Rs[i]};
    para_Pose[i][3] = q.x();
    para_Pose[i][4] = q.y();
    para_Pose[i][5] = q.z();
    para_Pose[i][6] = q.w();

    para_SpeedBias[i][0] = Vs[i].x();
    para_SpeedBias[i][1] = Vs[i].y();
    para_SpeedBias[i][2] = Vs[i].z();

    para_SpeedBias[i][3] = Bas[i].x();
    para_SpeedBias[i][4] = Bas[i].y();
    para_SpeedBias[i][5] = Bas[i].z();

    para_SpeedBias[i][6] = Bgs[i].x();
    para_SpeedBias[i][7] = Bgs[i].y();
    para_SpeedBias[i][8] = Bgs[i].z();

    // for VILO_FUSION_TYPE == 2
    for (int j = 0; j < NUM_LEG; j++) {
      para_FootBias[i][j][0] = Bfs[i][j].x();
      para_FootBias[i][j][1] = Bfs[i][j].y();
      para_FootBias[i][j][2] = Bfs[i][j].z();

      para_FootBias[i][j][3] = Bvs[i][j].x();
      para_FootBias[i][j][4] = Bvs[i][j].y();
      para_FootBias[i][j][5] = Bvs[i][j].z();

      para_FootBias[i][j][6] = Rhos[i][j](0);
    }
  }

  for (int i = 0; i < NUM_OF_CAM; i++) {
    para_Ex_Pose[i][0] = tic[i].x();
    para_Ex_Pose[i][1] = tic[i].y();
    para_Ex_Pose[i][2] = tic[i].z();
    Quaterniond q{ric[i]};
    para_Ex_Pose[i][3] = q.x();
    para_Ex_Pose[i][4] = q.y();
    para_Ex_Pose[i][5] = q.z();
    para_Ex_Pose[i][6] = q.w();
  }

  VectorXd dep = feature_manager_->getDepthVector();
  for (int i = 0; i < feature_manager_->getFeatureCount(); i++) para_Feature[i][0] = dep(i);

  para_Td[0][0] = td;
}

void VILOEstimator::double2vector() {
  Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
  Vector3d origin_P0 = Ps[0];

  if (failure_occur) {
    origin_R0 = Utility::R2ypr(last_R0);
    origin_P0 = last_P0;
    failure_occur = 0;
  }

  Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6], para_Pose[0][3], para_Pose[0][4], para_Pose[0][5]).toRotationMatrix());
  double y_diff = origin_R0.x() - origin_R00.x();
  // TODO
  Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
  if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0) {
    ROS_DEBUG("euler singular point!");
    rot_diff = Rs[0] * Quaterniond(para_Pose[0][6], para_Pose[0][3], para_Pose[0][4], para_Pose[0][5]).toRotationMatrix().transpose();
  }

  for (int i = 0; i <= WINDOW_SIZE; i++) {
    Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

    Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0], para_Pose[i][1] - para_Pose[0][1], para_Pose[i][2] - para_Pose[0][2]) +
            origin_P0;

    Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0], para_SpeedBias[i][1], para_SpeedBias[i][2]);

    Bas[i] = Vector3d(para_SpeedBias[i][3], para_SpeedBias[i][4], para_SpeedBias[i][5]);

    Bgs[i] = Vector3d(para_SpeedBias[i][6], para_SpeedBias[i][7], para_SpeedBias[i][8]);

    // for VILO_FUSION_TYPE == 2
    for (int j = 0; j < NUM_LEG; j++) {
      Bfs[i][j] = Vector3d(para_FootBias[i][j][0], para_FootBias[i][j][1], para_FootBias[i][j][2]);
      Bvs[i][j] = Vector3d(para_FootBias[i][j][3], para_FootBias[i][j][4], para_FootBias[i][j][5]);
      Rhos[i][j](0) = para_FootBias[i][j][6];
    }
  }

  for (int i = 0; i < NUM_OF_CAM; i++) {
    tic[i] = Vector3d(para_Ex_Pose[i][0], para_Ex_Pose[i][1], para_Ex_Pose[i][2]);
    ric[i] = Quaterniond(para_Ex_Pose[i][6], para_Ex_Pose[i][3], para_Ex_Pose[i][4], para_Ex_Pose[i][5]).normalized().toRotationMatrix();
  }

  VectorXd dep = feature_manager_->getDepthVector();
  for (int i = 0; i < feature_manager_->getFeatureCount(); i++) dep(i) = para_Feature[i][0];
  feature_manager_->setDepth(dep);

  td = para_Td[0][0];
}

bool VILOEstimator::failureDetection() {
  return false;
  if (feature_manager_->last_track_num < 2) {
    ROS_INFO(" little feature %d", feature_manager_->last_track_num);
    // return true;
  }
  if (Bas[WINDOW_SIZE].norm() > 2.5) {
    ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
    return true;
  }
  if (Bgs[WINDOW_SIZE].norm() > 1.0) {
    ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
    return true;
  }
  /*
  if (tic(0) > 1)
  {
      ROS_INFO(" big extri param estimation %d", tic(0) > 1);
      return true;
  }
  */
  Vector3d tmp_P = Ps[WINDOW_SIZE];
  if ((tmp_P - last_P).norm() > 5) {
    // ROS_INFO(" big translation");
    // return true;
  }
  if (abs(tmp_P.z() - last_P.z()) > 1) {
    // ROS_INFO(" big z translation");
    // return true;
  }
  Matrix3d tmp_R = Rs[WINDOW_SIZE];
  Matrix3d delta_R = tmp_R.transpose() * last_R;
  Quaterniond delta_Q(delta_R);
  double delta_angle;
  delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
  if (delta_angle > 50) {
    ROS_INFO(" big delta_angle ");
    // return true;
  }
  return false;
}

// main optimization functions
void VILOEstimator::slideWindow() {
  TicToc t_margin;
  if (marginalization_flag == MARGIN_OLD) {
    double t_0 = Headers[0];
    back_R0 = Rs[0];
    back_P0 = Ps[0];
    if (frame_count == WINDOW_SIZE) {
      for (int i = 0; i < WINDOW_SIZE; i++) {
        Headers[i] = Headers[i + 1];
        Rs[i].swap(Rs[i + 1]);
        Ps[i].swap(Ps[i + 1]);

        std::swap(pre_integrations[i], pre_integrations[i + 1]);

        dt_buf[i].swap(dt_buf[i + 1]);
        linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
        angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

        if (VILO_FUSION_TYPE == 1) {
          std::swap(lo_pre_integrations[i], lo_pre_integrations[i + 1]);
          lo_dt_buf[i].swap(lo_dt_buf[i + 1]);
          lo_velocity_buf[i].swap(lo_velocity_buf[i + 1]);
          lo_velocity_cov_buf[i].swap(lo_velocity_cov_buf[i + 1]);
        }

        Vs[i].swap(Vs[i + 1]);
        Bas[i].swap(Bas[i + 1]);
        Bgs[i].swap(Bgs[i + 1]);

        if (VILO_FUSION_TYPE == 2) {
          for (int j = 0; j < NUM_LEG; j++) {
            Bfs[i][j].swap(Bfs[i + 1][j]);
            Bvs[i][j].swap(Bvs[i + 1][j]);
            Rhos[i][j].swap(Rhos[i + 1][j]);
            std::swap(tlo_pre_integration[i][j], tlo_pre_integration[i + 1][j]);
            tight_lo_footGyr_buf[i][j].swap(tight_lo_footGyr_buf[i + 1][j]);
            tight_lo_jang_buf[i][j].swap(tight_lo_jang_buf[i + 1][j]);
            tight_lo_jvel_buf[i][j].swap(tight_lo_jvel_buf[i + 1][j]);
            tight_lo_dt_buf[i][j].swap(tight_lo_dt_buf[i + 1][j]);
            tight_lo_bodyGyr_buf[i][j].swap(tight_lo_bodyGyr_buf[i + 1][j]);
          }
        }
      }
      Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
      Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
      Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

      Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
      Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
      Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

      if (VILO_FUSION_TYPE == 2) {
        for (int j = 0; j < NUM_LEG; j++) {
          Bfs[WINDOW_SIZE][j] = Bfs[WINDOW_SIZE - 1][j];
          Bvs[WINDOW_SIZE][j] = Bvs[WINDOW_SIZE - 1][j];
          Rhos[WINDOW_SIZE][j] = Rhos[WINDOW_SIZE - 1][j];
        }
      }

      delete pre_integrations[WINDOW_SIZE];
      pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

      dt_buf[WINDOW_SIZE].clear();
      linear_acceleration_buf[WINDOW_SIZE].clear();
      angular_velocity_buf[WINDOW_SIZE].clear();

      if (VILO_FUSION_TYPE == 1) {
        delete lo_pre_integrations[WINDOW_SIZE];
        lo_pre_integrations[WINDOW_SIZE] = new LOIntegrationBase(lo_vel_0, lo_vel_cov_0);
        lo_dt_buf[WINDOW_SIZE].clear();
        lo_velocity_buf[WINDOW_SIZE].clear();
        lo_velocity_cov_buf[WINDOW_SIZE].clear();
      }

      if (VILO_FUSION_TYPE == 2) {
        for (int j = 0; j < NUM_LEG; j++) {
          delete tlo_pre_integration[WINDOW_SIZE][j];
          tlo_pre_integration[WINDOW_SIZE][j] = new LOTightIntegrationBase(
              j, tight_lo_joint_ang_0[j], tight_lo_joint_vel_0[j], tight_lo_body_gyr_0[j], tight_lo_foot_gyr_0[j], Bgs[WINDOW_SIZE],
              Bfs[WINDOW_SIZE][j], Bvs[WINDOW_SIZE][j], Rhos[WINDOW_SIZE][j], &lo_tight_utils_[j]);
        }
        for (int j = 0; j < NUM_LEG; j++) {
          tight_lo_dt_buf[WINDOW_SIZE][j].clear();
          tight_lo_bodyGyr_buf[WINDOW_SIZE][j].clear();
          tight_lo_footGyr_buf[WINDOW_SIZE][j].clear();
          tight_lo_jang_buf[WINDOW_SIZE][j].clear();
          tight_lo_jvel_buf[WINDOW_SIZE][j].clear();
        }
      }

      if (true || solver_flag == INITIAL) {
        map<double, ImageFrame>::iterator it_0;
        it_0 = all_image_frame.find(t_0);
        delete it_0->second.pre_integration;
        all_image_frame.erase(all_image_frame.begin(), it_0);
      }
      slideWindowOld();
    }
  } else {
    if (frame_count == WINDOW_SIZE) {
      Headers[frame_count - 1] = Headers[frame_count];
      Ps[frame_count - 1] = Ps[frame_count];
      Rs[frame_count - 1] = Rs[frame_count];

      for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++) {
        double tmp_dt = dt_buf[frame_count][i];
        Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
        Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

        pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

        dt_buf[frame_count - 1].push_back(tmp_dt);
        linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
        angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
      }

      Vs[frame_count - 1] = Vs[frame_count];
      Bas[frame_count - 1] = Bas[frame_count];
      Bgs[frame_count - 1] = Bgs[frame_count];

      if (VILO_FUSION_TYPE == 2) {
        for (int j = 0; j < NUM_LEG; j++) {
          Bfs[frame_count - 1][j] = Bfs[frame_count][j];
          Bvs[frame_count - 1][j] = Bvs[frame_count][j];
          Rhos[frame_count - 1][j] = Rhos[frame_count][j];
        }
      }
      delete pre_integrations[WINDOW_SIZE];
      pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

      dt_buf[WINDOW_SIZE].clear();
      linear_acceleration_buf[WINDOW_SIZE].clear();
      angular_velocity_buf[WINDOW_SIZE].clear();

      if (VILO_FUSION_TYPE == 1) {
        for (unsigned int i = 0; i < lo_dt_buf[frame_count].size(); i++) {
          double tmp_dt = lo_dt_buf[frame_count][i];
          Vector3d tmp_lo_vel = lo_velocity_buf[frame_count][i];
          Matrix3d tmp_lo_vel_cov = lo_velocity_cov_buf[frame_count][i];
          lo_pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_lo_vel, tmp_lo_vel_cov);
          lo_dt_buf[frame_count - 1].push_back(tmp_dt);
          lo_velocity_buf[frame_count - 1].push_back(tmp_lo_vel);
          lo_velocity_cov_buf[frame_count - 1].push_back(tmp_lo_vel_cov);
        }
        delete lo_pre_integrations[WINDOW_SIZE];
        lo_pre_integrations[WINDOW_SIZE] = new LOIntegrationBase{lo_vel_0, lo_vel_cov_0};
        lo_dt_buf[WINDOW_SIZE].clear();
        lo_velocity_buf[WINDOW_SIZE].clear();
        lo_velocity_cov_buf[WINDOW_SIZE].clear();
      }

      if (VILO_FUSION_TYPE == 2) {
        for (int j = 0; j < NUM_LEG; j++) {
          for (unsigned int i = 0; i < tight_lo_dt_buf[frame_count][j].size(); i++) {
            double tmp_tight_lo_dt = tight_lo_dt_buf[frame_count][j][i];
            Vector3d tmp_tight_lo_bodyGyr = tight_lo_bodyGyr_buf[frame_count][j][i];
            Vector3d tmp_tight_lo_footGyr = tight_lo_footGyr_buf[frame_count][j][i];
            Vector3d tmp_tight_lo_jang = tight_lo_jang_buf[frame_count][j][i];
            Vector3d tmp_tight_lo_jvel = tight_lo_jvel_buf[frame_count][j][i];
            tlo_pre_integration[frame_count - 1][j]->push_back(tmp_tight_lo_dt, tmp_tight_lo_bodyGyr, tmp_tight_lo_footGyr,
                                                               tmp_tight_lo_jang, tmp_tight_lo_jvel);

            tight_lo_dt_buf[frame_count - 1][j].push_back(tmp_tight_lo_dt);
            tight_lo_bodyGyr_buf[frame_count - 1][j].push_back(tmp_tight_lo_bodyGyr);
            tight_lo_footGyr_buf[frame_count - 1][j].push_back(tmp_tight_lo_footGyr);
            tight_lo_jang_buf[frame_count - 1][j].push_back(tmp_tight_lo_jang);
            tight_lo_jvel_buf[frame_count - 1][j].push_back(tmp_tight_lo_jvel);
          }
          delete tlo_pre_integration[WINDOW_SIZE][j];
          tlo_pre_integration[WINDOW_SIZE][j] = new LOTightIntegrationBase(
              j, tight_lo_joint_ang_0[j], tight_lo_joint_vel_0[j], tight_lo_body_gyr_0[j], tight_lo_foot_gyr_0[j], Bgs[WINDOW_SIZE],
              Bfs[WINDOW_SIZE][j], Bvs[WINDOW_SIZE][j], Rhos[WINDOW_SIZE][j], &lo_tight_utils_[j]);
          tight_lo_dt_buf[WINDOW_SIZE][j].clear();
          tight_lo_bodyGyr_buf[WINDOW_SIZE][j].clear();
          tight_lo_footGyr_buf[WINDOW_SIZE][j].clear();
          tight_lo_jang_buf[WINDOW_SIZE][j].clear();
          tight_lo_jvel_buf[WINDOW_SIZE][j].clear();
        }
      }

      slideWindowNew();
    }
  }
}

void VILOEstimator::slideWindowNew() {
  sum_of_front++;
  feature_manager_->removeFront(frame_count);
}

void VILOEstimator::slideWindowOld() {
  sum_of_back++;

  bool shift_depth = solver_flag == NON_LINEAR ? true : false;
  if (shift_depth) {
    Matrix3d R0, R1;
    Vector3d P0, P1;
    R0 = back_R0 * ric[0];
    R1 = Rs[0] * ric[0];
    P0 = back_P0 + back_R0 * tic[0];
    P1 = Ps[0] + Rs[0] * tic[0];
    feature_manager_->removeBackShiftDepth(R0, P0, R1, P1);
  } else
    feature_manager_->removeBack();
}
/*
 * key function : solve the optimization problem
 */
void VILOEstimator::optimization() {
  TicToc t_whole, t_prepare;
  // fill ceres variables
  vector2double();

  ceres::Problem problem;
  ceres::LossFunction* loss_function;
  // loss_function = NULL;
  loss_function = new ceres::HuberLoss(1.0);

  // add robot pose and velocity variables
  for (int i = 0; i < frame_count + 1; i++) {
    ceres::LocalParameterization* local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
    problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);

    for (int k = 0; k < NUM_LEG; k++) {
      problem.AddParameterBlock(para_FootBias[i][k], SIZE_FOOTBIAS);
    }
  }

  // maybe necessary
  // problem.SetParameterBlockConstant(para_Pose[0]);

  // kinematic parameters variables
  for (int i = 0; i < NUM_OF_CAM; i++) {
    ceres::LocalParameterization* local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
    if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation) {
      // ROS_INFO("estimate extinsic param");
      openExEstimation = 1;
    } else {
      // ROS_INFO("fix extinsic param");
      problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
  }
  problem.AddParameterBlock(para_Td[0], 1);
  if (!ESTIMATE_TD || Vs[0].norm() < 0.2) {
    problem.SetParameterBlockConstant(para_Td[0]);
  }

  // marginialization factor
  if (last_marginalization_info && last_marginalization_info->valid) {
    // construct new marginlization_factor
    MarginalizationFactor* marginalization_factor = new MarginalizationFactor(last_marginalization_info);
    problem.AddResidualBlock(marginalization_factor, NULL, last_marginalization_parameter_blocks);
  }

  // imu factor
  for (int i = 0; i < frame_count; i++) {
    int j = i + 1;
    if (pre_integrations[j]->sum_dt > 10.0) continue;
    IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
    problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
  }

  // lo factor
  if (VILO_FUSION_TYPE == 1) {
    for (int i = 0; i < frame_count; i++) {
      int j = i + 1;
      if (lo_pre_integrations[j]->sum_dt > 10.0) continue;
      LOFactor* lo_factor = new LOFactor(lo_pre_integrations[j]);
      problem.AddResidualBlock(lo_factor, NULL, para_Pose[i], para_Pose[j]);
    }
  }

  if (VILO_FUSION_TYPE == 2) {
    for (int i = 0; i < frame_count; i++) {
      int j = i + 1;
      for (int k = 0; k < NUM_LEG; k++) {
        if (tlo_pre_integration[j][k]->sum_dt > 10.0) continue;
        if (tlo_all_in_contact[j][k] == true) {
          LOTightFactor* tlo_factor = new LOTightFactor(tlo_pre_integration[j][k]);
          problem.AddResidualBlock(tlo_factor, NULL, para_Pose[i], para_SpeedBias[i], para_FootBias[i][k], para_Pose[j], para_SpeedBias[j],
                                   para_FootBias[j][k]);
        } else {
          LOConstantFactor* tlo_factor = new LOConstantFactor(k);
          problem.AddResidualBlock(tlo_factor, NULL, para_FootBias[i][k], para_FootBias[j][k]);
        }
      }
    }
  }

  // visual feature variables and factors
  int f_m_cnt = 0;
  int feature_index = -1;
  for (auto& it_per_id : feature_manager_->feature) {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < 4) continue;

    ++feature_index;

    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

    Vector3d pts_i = it_per_id.feature_per_frame[0].point;

    for (auto& it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      if (imu_i != imu_j) {
        Vector3d pts_j = it_per_frame.point;
        ProjectionTwoFrameOneCamFactor* f_td =
            new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                               it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
        problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index],
                                 para_Td[0]);
      }

      if (STEREO && it_per_frame.is_stereo) {
        Vector3d pts_j_right = it_per_frame.pointRight;
        if (imu_i != imu_j) {
          ProjectionTwoFrameTwoCamFactor* f =
              new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
          problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1],
                                   para_Feature[feature_index], para_Td[0]);
        } else {
          ProjectionOneFrameTwoCamFactor* f =
              new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
          problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
        }
      }
      f_m_cnt++;
    }
  }
  std::cout << "visual measurement count: " << f_m_cnt << std::endl;
  std::cout << "prepare for ceres: " << t_prepare.toc() << std::endl;

  // solve!
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  // options.num_threads = 2;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.max_num_iterations = NUM_ITERATIONS;
  // options.use_explicit_schur_complement = true;
  // options.minimizer_progress_to_stdout = true;
  // options.use_nonmonotonic_steps = true;
  if (marginalization_flag == MARGIN_OLD)
    options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
  else
    options.max_solver_time_in_seconds = SOLVER_TIME;
  TicToc t_solver;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // cout << summary.BriefReport() << endl;
  std::cout << "solver costs: " << t_solver.toc() << std::endl;

  // ceres output to state variables
  double2vector();

  // marginalization
  if (frame_count < WINDOW_SIZE) {
    return;
  } else {
    // perform marginalization
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD) {
      MarginalizationInfo* marginalization_info = new MarginalizationInfo();
      vector2double();

      if (last_marginalization_info && last_marginalization_info->valid) {
        vector<int> drop_set;
        for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
          if (last_marginalization_parameter_blocks[i] == para_Pose[0] || last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
            drop_set.push_back(i);

          for (int j = 0; j < NUM_LEG; j++) {
            if (last_marginalization_parameter_blocks[i] == para_FootBias[0][j]) {
              drop_set.push_back(i);
            }
          }
        }
        // construct new marginlization_factor
        MarginalizationFactor* marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        ResidualBlockInfo* residual_block_info =
            new ResidualBlockInfo(marginalization_factor, NULL, last_marginalization_parameter_blocks, drop_set);
        marginalization_info->addResidualBlockInfo(residual_block_info);
      }

      // imu factor
      if (pre_integrations[1]->sum_dt < 10.0) {
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
        ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(
            imu_factor, NULL, vector<double*>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]}, vector<int>{0, 1});
        marginalization_info->addResidualBlockInfo(residual_block_info);
      }
      // lo factor
      if (VILO_FUSION_TYPE == 1) {
        if (lo_pre_integrations[1]->sum_dt < 10.0) {
          LOFactor* lo_factor = new LOFactor(lo_pre_integrations[1]);
          ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(lo_factor, NULL,
                                                                         vector<double*>{
                                                                             para_Pose[0],
                                                                             para_Pose[1],
                                                                         },
                                                                         vector<int>{0});
          marginalization_info->addResidualBlockInfo(residual_block_info);
        }
      }

      if (VILO_FUSION_TYPE == 2) {
        for (int j = 0; j < NUM_LEG; j++) {
          if (tlo_pre_integration[1][j]->sum_dt < 10.0) {
            if (tlo_all_in_contact[1][j] == true) {
              LOTightFactor* tlo_factor = new LOTightFactor(tlo_pre_integration[1][j]);
              ResidualBlockInfo* residual_block_info =
                  new ResidualBlockInfo(tlo_factor, NULL,
                                        vector<double*>{para_Pose[0], para_SpeedBias[0], para_FootBias[0][j], para_Pose[1],
                                                        para_SpeedBias[1], para_FootBias[1][j]},
                                        vector<int>{0, 1, 2});
              marginalization_info->addResidualBlockInfo(residual_block_info);
            } else {
              LOConstantFactor* tlo_factor = new LOConstantFactor(j);
              ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(tlo_factor, NULL,
                                                                             vector<double*>{
                                                                                 para_FootBias[0][j],
                                                                                 para_FootBias[1][j],
                                                                             },
                                                                             vector<int>{0});
              marginalization_info->addResidualBlockInfo(residual_block_info);
            }
          }
        }
      }

      {
        int feature_index = -1;
        for (auto& it_per_id : feature_manager_->feature) {
          it_per_id.used_num = it_per_id.feature_per_frame.size();
          if (it_per_id.used_num < 4) continue;

          ++feature_index;

          int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
          if (imu_i != 0) continue;

          Vector3d pts_i = it_per_id.feature_per_frame[0].point;

          for (auto& it_per_frame : it_per_id.feature_per_frame) {
            imu_j++;
            if (imu_i != imu_j) {
              Vector3d pts_j = it_per_frame.point;
              ProjectionTwoFrameOneCamFactor* f_td =
                  new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                     it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
              ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(
                  f_td, loss_function,
                  vector<double*>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                  vector<int>{0, 3});
              marginalization_info->addResidualBlockInfo(residual_block_info);
            }
            if (STEREO && it_per_frame.is_stereo) {
              Vector3d pts_j_right = it_per_frame.pointRight;
              if (imu_i != imu_j) {
                ProjectionTwoFrameTwoCamFactor* f = new ProjectionTwoFrameTwoCamFactor(
                    pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                    it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                ResidualBlockInfo* residual_block_info =
                    new ResidualBlockInfo(f, loss_function,
                                          vector<double*>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1],
                                                          para_Feature[feature_index], para_Td[0]},
                                          vector<int>{0, 4});
                marginalization_info->addResidualBlockInfo(residual_block_info);
              } else {
                ProjectionOneFrameTwoCamFactor* f = new ProjectionOneFrameTwoCamFactor(
                    pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                    it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                ResidualBlockInfo* residual_block_info = new ResidualBlockInfo(
                    f, loss_function, vector<double*>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                    vector<int>{2});
                marginalization_info->addResidualBlockInfo(residual_block_info);
              }
            }
          }
        }
      }

      TicToc t_pre_margin;
      marginalization_info->preMarginalize();
      ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

      TicToc t_margin;
      marginalization_info->marginalize();
      ROS_DEBUG("marginalization %f ms", t_margin.toc());

      std::unordered_map<long, double*> addr_shift;
      for (int i = 1; i <= WINDOW_SIZE; i++) {
        addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        for (int j = 0; j < NUM_LEG; j++) {
          addr_shift[reinterpret_cast<long>(para_FootBias[i][j])] = para_FootBias[i - 1][j];
        }
      }
      for (int i = 0; i < NUM_OF_CAM; i++) addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

      addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

      vector<double*> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

      if (last_marginalization_info) delete last_marginalization_info;
      last_marginalization_info = marginalization_info;
      last_marginalization_parameter_blocks = parameter_blocks;

    } else {
      if (last_marginalization_info && std::count(std::begin(last_marginalization_parameter_blocks),
                                                  std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1])) {
        MarginalizationInfo* marginalization_info = new MarginalizationInfo();
        vector2double();
        if (last_marginalization_info && last_marginalization_info->valid) {
          vector<int> drop_set;
          for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
            ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
            if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1]) drop_set.push_back(i);
          }
          // construct new marginlization_factor
          MarginalizationFactor* marginalization_factor = new MarginalizationFactor(last_marginalization_info);
          ResidualBlockInfo* residual_block_info =
              new ResidualBlockInfo(marginalization_factor, NULL, last_marginalization_parameter_blocks, drop_set);

          marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        TicToc t_pre_margin;
        ROS_DEBUG("begin marginalization");
        marginalization_info->preMarginalize();
        ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

        TicToc t_margin;
        ROS_DEBUG("begin marginalization");
        marginalization_info->marginalize();
        ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

        std::unordered_map<long, double*> addr_shift;
        for (int i = 0; i <= WINDOW_SIZE; i++) {
          if (i == WINDOW_SIZE - 1)
            continue;
          else if (i == WINDOW_SIZE) {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];

            //
            for (int j = 0; j < NUM_LEG; j++) {
              addr_shift[reinterpret_cast<long>(para_FootBias[i][j])] = para_FootBias[i - 1][j];
            }

          } else {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];

            for (int j = 0; j < NUM_LEG; j++) {
              addr_shift[reinterpret_cast<long>(para_FootBias[i][j])] = para_FootBias[i][j];
            }
          }
        }
        for (int i = 0; i < NUM_OF_CAM; i++) addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double*> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
        if (last_marginalization_info) delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
      }
    }
    return;
  }
}

// get latest state
void VILOEstimator::updateLatestStates() {
  latest_time = Headers[frame_count] + td;
  latest_P = Ps[frame_count];
  latest_Q = Rs[frame_count];
  latest_V = Vs[frame_count];
  latest_Ba = Bas[frame_count];
  latest_Bg = Bgs[frame_count];
  latest_acc_0 = acc_0;
  latest_gyr_0 = gyr_0;

  return;
}

// feature tracking andd prediction helper functions
void VILOEstimator::getPoseInWorldFrame(Eigen::Matrix4d& T) {
  T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = Rs[frame_count];
  T.block<3, 1>(0, 3) = Ps[frame_count];
}

void VILOEstimator::getPoseInWorldFrame(int index, Eigen::Matrix4d& T) {
  T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = Rs[index];
  T.block<3, 1>(0, 3) = Ps[index];
}

void VILOEstimator::predictPtsInNextFrame() {
  // printf("predict pts in next frame\n");
  if (frame_count < 2) return;
  // predict next pose. Assume constant velocity motion
  Eigen::Matrix4d curT, prevT, nextT;
  getPoseInWorldFrame(curT);
  getPoseInWorldFrame(frame_count - 1, prevT);
  nextT = curT * (prevT.inverse() * curT);
  map<int, Eigen::Vector3d> predictPts;

  for (auto& it_per_id : feature_manager_->feature) {
    if (it_per_id.estimated_depth > 0) {
      int firstIndex = it_per_id.start_frame;
      int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
      // printf("cur frame index  %d last frame index %d\n", frame_count,
      // lastIndex);
      if ((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count) {
        double depth = it_per_id.estimated_depth;
        Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
        Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
        Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
        Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
        int ptsIndex = it_per_id.feature_id;
        predictPts[ptsIndex] = pts_cam;
      }
    }
  }
  feature_tracker_->setPrediction(predictPts);
  // printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double VILOEstimator::reprojectionError(Matrix3d& Ri, Vector3d& Pi, Matrix3d& rici, Vector3d& tici, Matrix3d& Rj, Vector3d& Pj,
                                        Matrix3d& ricj, Vector3d& ticj, double depth, Vector3d& uvi, Vector3d& uvj) {
  Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
  Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
  Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
  double rx = residual.x();
  double ry = residual.y();
  return sqrt(rx * rx + ry * ry);
}

void VILOEstimator::outliersRejection(set<int>& removeIndex) {
  // return;
  int feature_index = -1;
  for (auto& it_per_id : feature_manager_->feature) {
    double err = 0;
    int errCnt = 0;
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < 4) continue;
    feature_index++;
    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
    Vector3d pts_i = it_per_id.feature_per_frame[0].point;
    double depth = it_per_id.estimated_depth;
    for (auto& it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      if (imu_i != imu_j) {
        Vector3d pts_j = it_per_frame.point;
        double tmp_error =
            reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j], Ps[imu_j], ric[0], tic[0], depth, pts_i, pts_j);
        err += tmp_error;
        errCnt++;
        // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
      }

      if (it_per_frame.is_stereo) {
        Vector3d pts_j_right = it_per_frame.pointRight;
        if (imu_i != imu_j) {
          double tmp_error =
              reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j], Ps[imu_j], ric[1], tic[1], depth, pts_i, pts_j_right);
          err += tmp_error;
          errCnt++;
          // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
        } else {
          double tmp_error =
              reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j], Ps[imu_j], ric[1], tic[1], depth, pts_i, pts_j_right);
          err += tmp_error;
          errCnt++;
          // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
        }
      }
    }
    double ave_err = err / errCnt;
    if (ave_err * FOCAL_LENGTH > 3) removeIndex.insert(it_per_id.feature_id);
  }
}
