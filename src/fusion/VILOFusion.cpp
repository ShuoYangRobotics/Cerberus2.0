#include "fusion/VILOFusion.hpp"

VILOFusion::VILOFusion(ros::NodeHandle nh) {
  nh_ = nh;

  mipo_estimator = std::make_unique<MIPOEstimator>();
  sipo_estimator = std::make_unique<SIPOEstimator>();
  vilo_estimator = std::make_unique<VILOEstimator>();
  vilo_estimator->setParameter();

  // bind the po_loop_thread_ to the loop function
  po_loop_thread_ = std::thread(&VILOFusion::POLoop, this);
  vilo_loop_thread_ = std::thread(&VILOFusion::VILOLoop, this);

  // initialize callback functions
  imu_sub_ = nh_.subscribe(IMU_TOPIC, 1000, &VILOFusion::imuCallback, this);
  joint_foot_sub_ = nh_.subscribe(LEG_TOPIC, 1000, &VILOFusion::jointFootCallback, this);
  fl_imu_sub_ = nh_.subscribe(FL_IMU_TOPIC, 1000, &VILOFusion::flImuCallback, this);
  fr_imu_sub_ = nh_.subscribe(FR_IMU_TOPIC, 1000, &VILOFusion::frImuCallback, this);
  rl_imu_sub_ = nh_.subscribe(RL_IMU_TOPIC, 1000, &VILOFusion::rlImuCallback, this);
  rr_imu_sub_ = nh_.subscribe(RR_IMU_TOPIC, 1000, &VILOFusion::rrImuCallback, this);
  gt_sub_ = nh_.subscribe(GT_TOPIC, 1000, &VILOFusion::gtCallback, this);

  img0_sub_ = nh_.subscribe(IMAGE0_TOPIC, 1000, &VILOFusion::img0Callback, this);
  img1_sub_ = nh_.subscribe(IMAGE1_TOPIC, 1000, &VILOFusion::img1Callback, this);

  // initialize queues and specify their main types
  mq_imu_ = SWE::MeasureQueue(SWE::MeasureType::BODY_IMU);
  mq_joint_foot_ = SWE::MeasureQueue(SWE::MeasureType::LEG);
  mq_foot_force_ = SWE::MeasureQueue(SWE::MeasureType::FOOT_FORCE_ALL);
  mq_fl_imu_ = SWE::MeasureQueue(SWE::MeasureType::BODY_IMU);
  mq_fr_imu_ = SWE::MeasureQueue(SWE::MeasureType::BODY_IMU);
  mq_rl_imu_ = SWE::MeasureQueue(SWE::MeasureType::BODY_IMU);
  mq_rr_imu_ = SWE::MeasureQueue(SWE::MeasureType::BODY_IMU);
  mq_gt_ = SWE::MeasureQueue(SWE::MeasureType::DIRECT_POSE);

  // initialize publishers
  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/mipo/estimate_pose", 1000);
  pose_vilo_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/vilo/estimate_pose", 1000);
  twist_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/mipo/estimate_twist", 1000);
  twist_vilo_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/vilo/estimate_twist", 1000);

  // initialize the filter
  for (int i = 0; i < 3; i++) {
    fl_imu_acc_filter_[i] = MovingWindowFilter(FOOT_IMU_MOVMEAN_WINDOW_SIZE);
    fr_imu_acc_filter_[i] = MovingWindowFilter(FOOT_IMU_MOVMEAN_WINDOW_SIZE);
    rl_imu_acc_filter_[i] = MovingWindowFilter(FOOT_IMU_MOVMEAN_WINDOW_SIZE);
    rr_imu_acc_filter_[i] = MovingWindowFilter(FOOT_IMU_MOVMEAN_WINDOW_SIZE);
    fl_imu_gyro_filter_[i] = MovingWindowFilter(FOOT_IMU_MOVMEAN_WINDOW_SIZE);
    fr_imu_gyro_filter_[i] = MovingWindowFilter(FOOT_IMU_MOVMEAN_WINDOW_SIZE);
    rl_imu_gyro_filter_[i] = MovingWindowFilter(FOOT_IMU_MOVMEAN_WINDOW_SIZE);
    rr_imu_gyro_filter_[i] = MovingWindowFilter(FOOT_IMU_MOVMEAN_WINDOW_SIZE);
  }
  for (int i = 0; i < NUM_DOF; i++) {
    joint_foot_filter_[i] = MovingWindowFilter(JOINT_MOVMEAN_WINDOW_SIZE);
  }
  yaw_filter_ = MovingWindowFilter(YAW_MOVMEAN_WINDOW_SIZE);

  // init the output state other the write to file logic will segfault
  po_x = Eigen::VectorXd::Zero(SS_SIZE);
}

VILOFusion ::~VILOFusion() {
  po_loop_thread_.join();
  vilo_loop_thread_.join();
}

void VILOFusion::POLoop() {
  /*
   * The loop function do several things:
        sync stereo image and input them to VILOEstimator
        sync proprioceptive sensors
        run MIPOEstimator
        input necessary information to VILOEstimator
        record MIPO and VILO results
   */
  const double LOOP_DT = 2.5;  // 400Hz
  prev_loop_time = ros::Time::now().toSec();
  prev_esti_time = ros::Time::now().toSec();

  std::shared_ptr<MIPOEstimatorSensorData> prev_data = nullptr;
  std::shared_ptr<MIPOEstimatorSensorData> curr_data = nullptr;

  std::shared_ptr<SIPOEstimatorSensorData> sipo_prev_data = nullptr;
  std::shared_ptr<SIPOEstimatorSensorData> sipo_curr_data = nullptr;

  while (ros::ok()) {
    /* record start time */
    auto loop_start = std::chrono::system_clock::now();
    auto ros_loop_start = ros::Time::now().toSec();

    /* get a bunch of ros objects */
    double curr_loop_time = ros::Time::now().toSec();
    double dt_ros = curr_loop_time - prev_loop_time;
    prev_loop_time = curr_loop_time;
    // std::cout << "dt_ros: " << dt_ros << std::endl;
    if (dt_ros == 0) {
      continue;
    }

    /* estimator logic */

    // process PO data and
    if (KF_TYPE == 0) {
      // run MIPO
      if (isMIPODataAvailable()) {
        Eigen::Matrix<double, 55, 1> sensor_data;
        double curr_esti_time = interpolateMIPOData(sensor_data, dt_ros);
        // std::cout << curr_esti_time << std::endl;
        // run MIPO
        if (prev_data == nullptr) {
          prev_data = std::make_shared<MIPOEstimatorSensorData>();
          prev_data->loadFromVec(sensor_data);
          // initialize the estimator
          mipo_x = mipo_estimator->ekfInitState(*prev_data);
          mipo_x(mipo_x.size() - 1) = curr_esti_time;  // the inital time stamp
          mipo_P = 1e-4 * Eigen::Matrix<double, MS_SIZE, MS_SIZE>::Identity();
        } else {
          curr_data = std::make_shared<MIPOEstimatorSensorData>();
          curr_data->loadFromVec(sensor_data);

          // do estimation
          Eigen::Matrix<double, MS_SIZE, 1> mipo_x_k1_est;
          Eigen::Matrix<double, MS_SIZE, MS_SIZE> mipo_P_k1_est;
          mipo_estimator->ekfUpdate(mipo_x, mipo_P, *prev_data, *curr_data, dt_ros, mipo_x_k1_est, mipo_P_k1_est);

          mipo_x = mipo_x_k1_est;
          // std::cout << "x: " << x.transpose() << std::endl;
          mipo_P = mipo_P_k1_est;
          prev_data = curr_data;
          // publish the estimation result
          Eigen::VectorXd x = Eigen::VectorXd::Map(mipo_x.data(), mipo_x.size());
          Eigen::MatrixXd P = mipo_P.cast<double>();
          publishPOEstimationResult(x, P);

          // inputPODataToVILO();
          vilo_estimator->inputBodyIMU(curr_esti_time, curr_data->body_acc, curr_data->body_gyro);
          // input LO velocity
          if (VILO_FUSION_TYPE == 1) {
            vilo_estimator->inputLOVel(curr_esti_time, mipo_x.segment<3>(3));
          }
        }
        po_x = mipo_x;
      } else {
        prev_esti_time += dt_ros;
      }
    } else {
      // run SIPO
      if (isSIPODataAvailable()) {
        Eigen::Matrix<double, 35, 1> sensor_data;
        double curr_esti_time = interpolateSIPOData(sensor_data, dt_ros);
        std::cout << curr_esti_time << std::endl;
        // run SIPO
        if (sipo_prev_data == nullptr) {
          sipo_prev_data = std::make_shared<SIPOEstimatorSensorData>();
          sipo_prev_data->loadFromVec(sensor_data);
          // initialize the estimator
          sipo_x = sipo_estimator->ekfInitState(*sipo_prev_data);
          sipo_x(sipo_x.size() - 1) = curr_esti_time;  // the inital time stamp
          sipo_P = 1e-4 * Eigen::Matrix<double, SS_SIZE, SS_SIZE>::Identity();
        } else {
          sipo_curr_data = std::make_shared<SIPOEstimatorSensorData>();
          sipo_curr_data->loadFromVec(sensor_data);

          // do estimation
          Eigen::Matrix<double, SS_SIZE, 1> sipo_x_k1_est;
          Eigen::Matrix<double, SS_SIZE, SS_SIZE> sipo_P_k1_est;
          sipo_estimator->ekfUpdate(sipo_x, sipo_P, *sipo_prev_data, *sipo_curr_data, dt_ros, sipo_x_k1_est, sipo_P_k1_est);

          sipo_x = sipo_x_k1_est;
          // std::cout << "x: " << x.transpose() << std::endl;
          sipo_P = sipo_P_k1_est;
          sipo_prev_data = sipo_curr_data;
          // publish the estimation result
          Eigen::VectorXd x = Eigen::VectorXd::Map(sipo_x.data(), sipo_x.size());
          Eigen::MatrixXd P = sipo_P.cast<double>();
          publishPOEstimationResult(x, P);

          // inputPODataToVILO();
          vilo_estimator->inputBodyIMU(curr_esti_time, sipo_curr_data->body_acc, sipo_curr_data->body_gyro);
          // input LO velocity
          if (VILO_FUSION_TYPE == 1) {
            vilo_estimator->inputLOVel(curr_esti_time, sipo_x.segment<3>(3));
          }
        }
        po_x = sipo_x;
      } else {
        prev_esti_time += dt_ros;
      }
    }
    // extract the result from VILO estimator and publish it
    vilo_x = vilo_estimator->outputState();
    publishVILOEstimationResult(vilo_x);

    /* record end time */
    auto loop_end = std::chrono::system_clock::now();
    auto loop_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start);
    // std::cout << "total loop time: " << loop_elapsed.count() << std::endl;

    auto ros_loop_end = ros::Time::now().toSec();
    auto ros_loop_elapsed = ros_loop_end - ros_loop_start;

    /* sleep a while to make sure the loop run at LOOP_DT */
    if (ros_loop_elapsed * 1000 >= LOOP_DT) {
      // do not sleep
      // std::cout
      //     << "POLoop computation time is longer than desired dt, optimize "
      //        "code or increase LOOP_DT"
      //     << std::endl;
    } else {
      double sleep_time = LOOP_DT * 0.001 - ros_loop_elapsed;
      ros::Duration(sleep_time).sleep();
    }
  }
  return;
}

void VILOFusion::VILOLoop() {
  // loop runs a little over 20Hz, call inputImagesToVILO() every 50ms
  const double LOOP_DT = 20;  // 20Hz
  int loop_count = 0;
  while (ros::ok()) {
    loop_count++;

    auto ros_loop_start = ros::Time::now().toSec();
    /* logic */
    inputImagesToVILO();

    // logging here.
    if (loop_count % 4) {  // 1/4 of the loop

      // write VILO
      double vilo_time = vilo_x(0);
      Eigen::Vector3d vilo_pos = vilo_x.segment<3>(1);
      Eigen::Vector3d vilo_vel = vilo_x.segment<3>(8);
      Eigen::VectorXd vilo_quat_vec = vilo_x.segment<4>(4);  // x y z w
      Eigen::Quaterniond vilo_quat(vilo_quat_vec(3), vilo_quat_vec(0), vilo_quat_vec(1), vilo_quat_vec(2));
      Eigen::Vector3d vilo_euler = legged::quat_to_euler(vilo_quat);
      ofstream fout_vilo(VILO_RESULT_PATH, ios::app);
      fout_vilo.setf(ios::fixed, ios::floatfield);
      fout_vilo.precision(15);
      fout_vilo << vilo_time << ",";
      fout_vilo.precision(5);
      fout_vilo << vilo_pos(0) << "," << vilo_pos(1) << "," << vilo_pos(2) << ",";
      fout_vilo << vilo_euler(0) << "," << vilo_euler(1) << "," << vilo_euler(2) << ",";
      fout_vilo << vilo_vel(0) << "," << vilo_vel(1) << "," << vilo_vel(2);
      fout_vilo << endl;
      fout_vilo.close();

      // write MIPO
      if (KF_TYPE == 0) {
        double mipo_time = mipo_x(mipo_x.size() - 1);
        Eigen::Vector3d mipo_pos = mipo_x.segment<3>(0);
        Eigen::Vector3d mipo_vel = mipo_x.segment<3>(3);
        Eigen::Vector3d mipo_euler = mipo_x.segment<3>(6);
        ofstream fout_mipo(PO_RESULT_PATH, ios::app);
        fout_mipo.setf(ios::fixed, ios::floatfield);
        fout_mipo.precision(15);
        fout_mipo << mipo_time << ",";
        fout_mipo.precision(5);
        fout_mipo << mipo_pos(0) << "," << mipo_pos(1) << "," << mipo_pos(2) << ",";
        fout_mipo << mipo_euler(0) << "," << mipo_euler(1) << "," << mipo_euler(2) << ",";
        fout_mipo << mipo_vel(0) << "," << mipo_vel(1) << "," << mipo_vel(2);
        fout_mipo << endl;
        fout_mipo.close();
      } else {
        double sipo_time = sipo_x(sipo_x.size() - 1);
        Eigen::Vector3d sipo_pos = sipo_x.segment<3>(0);
        Eigen::Vector3d sipo_vel = sipo_x.segment<3>(3);
        Eigen::Vector3d sipo_euler = sipo_x.segment<3>(6);
        ofstream fout_sipo(PO_RESULT_PATH, ios::app);
        fout_sipo.setf(ios::fixed, ios::floatfield);
        fout_sipo.precision(15);
        fout_sipo << sipo_time << ",";
        fout_sipo.precision(5);
        fout_sipo << sipo_pos(0) << "," << sipo_pos(1) << "," << sipo_pos(2) << ",";
        fout_sipo << sipo_euler(0) << "," << sipo_euler(1) << "," << sipo_euler(2) << ",";
        fout_sipo << sipo_vel(0) << "," << sipo_vel(1) << "," << sipo_vel(2);
        fout_sipo << endl;
        fout_sipo.close();
      }

      // write GT is mocap is available
      if (mq_gt_.size() > 0) {
        std::shared_ptr<SWE::Measurement> gt_meas = mq_gt_.top();
        if (gt_meas != nullptr) {  // if gt is available
          latest_gt_meas = gt_meas;

          double gt_time = gt_meas->getTime();
          Eigen::VectorXd gt_data = latest_gt_meas->getVector();
          Eigen::VectorXd gt_quat_vec = gt_data.segment<4>(3);
          Eigen::Quaterniond gt_quat(gt_quat_vec(0), gt_quat_vec(1), gt_quat_vec(2), gt_quat_vec(3));

          Eigen::Vector3d gt_pos = gt_data.segment<3>(0);
          Eigen::Vector3d gt_euler = legged::quat_to_euler<double>(gt_quat);
          ofstream fout_gt(GT_RESULT_PATH, ios::app);
          fout_gt.setf(ios::fixed, ios::floatfield);
          fout_gt.precision(15);
          fout_gt << gt_time << ",";
          fout_gt.precision(5);
          fout_gt << gt_pos(0) << "," << gt_pos(1) << "," << gt_pos(2) << ",";
          fout_gt << gt_euler(0) << "," << gt_euler(1) << "," << gt_euler(2) << ",";
          fout_gt << 0 << "," << 0 << "," << 0;
          fout_gt << endl;
          fout_gt.close();
        }
      }
    }

    auto ros_loop_end = ros::Time::now().toSec();
    auto ros_loop_elapsed = ros_loop_end - ros_loop_start;
    /* sleep a while to make sure the loop run at LOOP_DT */
    if (ros_loop_elapsed * 1000 >= LOOP_DT) {
      // do not sleep
      // std::cout
      //     << "VILOLoop computation time is longer than desired dt, optimize "
      //        "code or increase LOOP_DT"
      //     << std::endl;
    } else {
      double sleep_time = LOOP_DT * 0.001 - ros_loop_elapsed;
      ros::Duration(sleep_time).sleep();
    }
  }
}

void VILOFusion::publishPOEstimationResult(Eigen::VectorXd& x, Eigen::MatrixXd& P) {
  // extract position from the first 3 elements of x
  Eigen::Vector3d pos = x.segment<3>(0);
  // extract position uncertainty from the first 3 elements of P
  Eigen::Matrix3d pos_cov = P.block<3, 3>(0, 0);

  // extract velocity from the next 3 elements of x
  Eigen::Vector3d vel = x.segment<3>(3);
  // extract velocity uncertainty from the next 3 elements of P
  Eigen::Matrix3d vel_cov = P.block<3, 3>(3, 3);

  // extract euler angles from the next 3 elements of x
  Eigen::Vector3d euler = x.segment<3>(6);
  // extract euler angles uncertainty from the next 3 elements of P
  Eigen::Matrix3d euler_cov = P.block<3, 3>(6, 6);

  // order is w, x, y, z
  Eigen::Matrix<double, 4, 1> quat_vec = legged::euler_to_quat(euler);
  // convert to Eigen::Quaterniond
  Eigen::Quaterniond quat(quat_vec(0), quat_vec(1), quat_vec(2), quat_vec(3));

  // publish position, velocity, quaternion orientation
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "world";
  pose_msg.pose.pose.position.x = pos(0);
  pose_msg.pose.pose.position.y = pos(1);
  pose_msg.pose.pose.position.z = pos(2);
  pose_msg.pose.pose.orientation.w = quat.w();
  pose_msg.pose.pose.orientation.x = quat.x();
  pose_msg.pose.pose.orientation.y = quat.y();
  pose_msg.pose.pose.orientation.z = quat.z();
  pose_msg.pose.covariance[0] = pos_cov(0, 0);
  pose_msg.pose.covariance[1] = pos_cov(0, 1);
  pose_msg.pose.covariance[2] = pos_cov(0, 2);
  pose_msg.pose.covariance[6] = pos_cov(1, 0);
  pose_msg.pose.covariance[7] = pos_cov(1, 1);
  pose_msg.pose.covariance[8] = pos_cov(1, 2);
  pose_msg.pose.covariance[12] = pos_cov(2, 0);
  pose_msg.pose.covariance[13] = pos_cov(2, 1);
  pose_msg.pose.covariance[14] = pos_cov(2, 2);
  pose_pub_.publish(pose_msg);

  geometry_msgs::TwistWithCovarianceStamped twist_msg;
  twist_msg.header.stamp = ros::Time::now();
  twist_msg.header.frame_id = "world";
  twist_msg.twist.twist.linear.x = vel(0);
  twist_msg.twist.twist.linear.y = vel(1);
  twist_msg.twist.twist.linear.z = vel(2);

  twist_pub_.publish(twist_msg);
  return;
}

void VILOFusion::publishVILOEstimationResult(Eigen::Matrix<double, VS_OUTSIZE, 1>& state) {
  Eigen::Vector3d pos = state.segment(1, 3);
  Eigen::Quaterniond quat(state(7), state(4), state(5),
                          state(6));  // state quaternion is in order x, y, z, w
                                      // because of Eigen coeffs function
  Eigen::Vector3d vel = state.segment(8, 3);
  // publish pose topic and twist topic
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "world";
  pose_msg.pose.pose.position.x = pos(0);
  pose_msg.pose.pose.position.y = pos(1);
  pose_msg.pose.pose.position.z = pos(2);
  pose_msg.pose.pose.orientation.w = quat.w();
  pose_msg.pose.pose.orientation.x = quat.x();
  pose_msg.pose.pose.orientation.y = quat.y();
  pose_msg.pose.pose.orientation.z = quat.z();
  for (int i = 0; i < 36; i++) {
    pose_msg.pose.covariance[i] = 0;
  }
  pose_vilo_pub_.publish(pose_msg);

  geometry_msgs::TwistWithCovarianceStamped twist_msg;
  twist_msg.header.stamp = ros::Time::now();
  twist_msg.header.frame_id = "world";
  twist_msg.twist.twist.linear.x = vel(0);
  twist_msg.twist.twist.linear.y = vel(1);
  twist_msg.twist.twist.linear.z = vel(2);

  for (int i = 0; i < 36; i++) {
    twist_msg.twist.covariance[i] = 0;
  }
  twist_vilo_pub_.publish(twist_msg);

  // printf("time: %f, t: %f %f %f q: %f %f %f %f \n", ros::Time::now().toSec(),
  //        pos(0), pos(1), pos(2), quat.w(), quat.x(), quat.y(), quat.z());
  return;
}

// private functions
void VILOFusion::img0Callback(const sensor_msgs::ImageConstPtr& img_msg) {
  const std::lock_guard<std::mutex> lock(mtx_image);
  img0_buf.push(img_msg);
}

void VILOFusion::img1Callback(const sensor_msgs::ImageConstPtr& img_msg) {
  const std::lock_guard<std::mutex> lock(mtx_image);
  img1_buf.push(img_msg);
}
cv::Mat VILOFusion::getImageFromMsg(const sensor_msgs::ImageConstPtr& img_msg) {
  cv_bridge::CvImageConstPtr ptr;
  if (img_msg->encoding == "8UC1") {
    sensor_msgs::Image img;
    img.header = img_msg->header;
    img.height = img_msg->height;
    img.width = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step = img_msg->step;
    img.data = img_msg->data;
    img.encoding = "mono8";
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  } else
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

  cv::Mat img = ptr->image.clone();
  return img;
}

void VILOFusion::inputImagesToVILO() {
  const std::lock_guard<std::mutex> lock(mtx_image);
  cv::Mat image0, image1;
  std_msgs::Header header;
  double time = 0;
  if (!img0_buf.empty() && !img1_buf.empty()) {
    double time0 = img0_buf.front()->header.stamp.toSec();
    double time1 = img1_buf.front()->header.stamp.toSec();
    // 0.003s sync tolerance
    if (time0 < time1 - 0.003) {
      img0_buf.pop();
      std::cout << "throw img0" << std::endl;
    } else if (time0 > time1 + 0.003) {
      img1_buf.pop();
      std::cout << "throw img1" << std::endl;
    } else {
      time = img0_buf.front()->header.stamp.toSec();
      header = img0_buf.front()->header;
      image0 = getImageFromMsg(img0_buf.front());
      img0_buf.pop();
      image1 = getImageFromMsg(img1_buf.front());
      img1_buf.pop();
      // std::cout << "synced image, " << std::setprecision(15) << time0 <<
      // std::endl;
      // we have 60ms budget for one VILO solve
      vilo_estimator->inputImage(time, image0, image1);
    }
  }
}

// callback functions
void VILOFusion::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  double t = msg->header.stamp.toSec();
  // assemble sensor data
  Eigen::Vector3d acc = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  Eigen::Vector3d ang_vel = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  mtx.lock();
  mq_imu_.push(t, acc, ang_vel);

  // limit the size of the message queue
  if (!mq_imu_.empty() && mq_imu_.size() > MAX_PO_QUEUE_SIZE) {
    mq_imu_.pop();
  }
  mtx.unlock();

  return;
}

void VILOFusion::jointFootCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  double t = msg->header.stamp.toSec();
  Eigen::Matrix<double, 12, 1> joint_pos;
  Eigen::Matrix<double, 12, 1> joint_vel;
  Eigen::Matrix<double, NUM_LEG, 1> foot_force;

  for (int i = 0; i < NUM_DOF; i++) {
    joint_pos(i) = msg->position[i];
    joint_vel(i) = joint_foot_filter_[i].CalculateAverage(msg->velocity[i]);
  }
  // TODO: record foot force as well
  for (int i = 0; i < NUM_LEG; i++) {
    foot_force(i) = msg->effort[NUM_DOF + i];
  }

  mtx.lock();
  mq_joint_foot_.push(t, joint_pos, joint_vel);
  // limit the size of the message queue
  if (!mq_joint_foot_.empty() && mq_joint_foot_.size() > MAX_PO_QUEUE_SIZE) {
    mq_joint_foot_.pop();
  }
  mq_foot_force_.push(t - 0.03, foot_force);  // notice: a delay of 30ms
  // limit the size of the message queue
  if (!mq_foot_force_.empty() && mq_foot_force_.size() > MAX_PO_QUEUE_SIZE) {
    mq_foot_force_.pop();
  }
  mtx.unlock();

  return;
}

void VILOFusion::flImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  double t = msg->header.stamp.toSec() - FOOT_IMU_DELAY;
  // assemble sensor data
  Eigen::Vector3d acc = Eigen::Vector3d(fl_imu_acc_filter_[0].CalculateAverage(msg->linear_acceleration.x),
                                        fl_imu_acc_filter_[1].CalculateAverage(msg->linear_acceleration.y),
                                        fl_imu_acc_filter_[2].CalculateAverage(msg->linear_acceleration.z));
  Eigen::Vector3d ang_vel = Eigen::Vector3d(fl_imu_gyro_filter_[0].CalculateAverage(msg->angular_velocity.x),
                                            fl_imu_gyro_filter_[1].CalculateAverage(msg->angular_velocity.y),
                                            fl_imu_gyro_filter_[2].CalculateAverage(msg->angular_velocity.z));

  mtx.lock();
  mq_fl_imu_.push(t, acc, ang_vel, 0);
  // limit the size of the message queue
  if (!mq_fl_imu_.empty() && mq_fl_imu_.size() > MAX_PO_QUEUE_SIZE) {
    mq_fl_imu_.pop();
  }
  mtx.unlock();

  return;
}

void VILOFusion::frImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  double t = msg->header.stamp.toSec() - FOOT_IMU_DELAY;
  // assemble sensor data
  Eigen::Vector3d acc = Eigen::Vector3d(fr_imu_acc_filter_[0].CalculateAverage(msg->linear_acceleration.x),
                                        fr_imu_acc_filter_[1].CalculateAverage(msg->linear_acceleration.y),
                                        fr_imu_acc_filter_[2].CalculateAverage(msg->linear_acceleration.z));
  Eigen::Vector3d ang_vel = Eigen::Vector3d(fr_imu_gyro_filter_[0].CalculateAverage(msg->angular_velocity.x),
                                            fr_imu_gyro_filter_[1].CalculateAverage(msg->angular_velocity.y),
                                            fr_imu_gyro_filter_[2].CalculateAverage(msg->angular_velocity.z));

  mtx.lock();
  mq_fr_imu_.push(t, acc, ang_vel, 1);
  // limit the size of the message queue
  if (!mq_fr_imu_.empty() && mq_fr_imu_.size() > MAX_PO_QUEUE_SIZE) {
    mq_fr_imu_.pop();
  }
  mtx.unlock();

  return;
}

void VILOFusion::rlImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  double t = msg->header.stamp.toSec() - FOOT_IMU_DELAY;
  // assemble sensor data
  Eigen::Vector3d acc = Eigen::Vector3d(rl_imu_acc_filter_[0].CalculateAverage(msg->linear_acceleration.x),
                                        rl_imu_acc_filter_[1].CalculateAverage(msg->linear_acceleration.y),
                                        rl_imu_acc_filter_[2].CalculateAverage(msg->linear_acceleration.z));
  Eigen::Vector3d ang_vel = Eigen::Vector3d(rl_imu_gyro_filter_[0].CalculateAverage(msg->angular_velocity.x),
                                            rl_imu_gyro_filter_[1].CalculateAverage(msg->angular_velocity.y),
                                            rl_imu_gyro_filter_[2].CalculateAverage(msg->angular_velocity.z));

  mtx.lock();
  mq_rl_imu_.push(t, acc, ang_vel, 2);
  // limit the size of the message queue
  if (!mq_rl_imu_.empty() && mq_rl_imu_.size() > MAX_PO_QUEUE_SIZE) {
    mq_rl_imu_.pop();
  }
  mtx.unlock();

  return;
}

void VILOFusion::rrImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  double t = msg->header.stamp.toSec() - FOOT_IMU_DELAY;
  // assemble sensor data
  Eigen::Vector3d acc = Eigen::Vector3d(rr_imu_acc_filter_[0].CalculateAverage(msg->linear_acceleration.x),
                                        rr_imu_acc_filter_[1].CalculateAverage(msg->linear_acceleration.y),
                                        rr_imu_acc_filter_[2].CalculateAverage(msg->linear_acceleration.z));
  Eigen::Vector3d ang_vel = Eigen::Vector3d(rr_imu_gyro_filter_[0].CalculateAverage(msg->angular_velocity.x),
                                            rr_imu_gyro_filter_[1].CalculateAverage(msg->angular_velocity.y),
                                            rr_imu_gyro_filter_[2].CalculateAverage(msg->angular_velocity.z));

  mtx.lock();
  mq_rr_imu_.push(t, acc, ang_vel, 3);
  // limit the size of the message queue
  if (!mq_rr_imu_.empty() && mq_rr_imu_.size() > MAX_PO_QUEUE_SIZE) {
    mq_rr_imu_.pop();
  }
  mtx.unlock();

  return;
}

// also receive and record the ground truth for training/debuging purpose
void VILOFusion::gtCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  double t = msg->header.stamp.toSec();
  // get position and orientation
  Eigen::Vector3d pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

  mq_gt_.push(t, pos, q);
  // limit the size of the message queue
  if (!mq_gt_.empty() && mq_gt_.size() > MAX_PO_QUEUE_SIZE) {
    mq_gt_.pop();
  }

  // if we have received the ground truth, we set the flag to true
  is_gt_available_ = true;
  return;
}

// functions for processing PO data
bool VILOFusion::isMIPODataAvailable() {
  if (mq_imu_.size() > MIN_PO_QUEUE_SIZE && mq_joint_foot_.size() > MIN_PO_QUEUE_SIZE && mq_fl_imu_.size() > MIN_PO_QUEUE_SIZE &&
      mq_fr_imu_.size() > MIN_PO_QUEUE_SIZE && mq_rl_imu_.size() > MIN_PO_QUEUE_SIZE && mq_rr_imu_.size() > MIN_PO_QUEUE_SIZE) {
    return true;
  }
  return false;
}

bool VILOFusion::isSIPODataAvailable() {
  if (mq_imu_.size() > MIN_PO_QUEUE_SIZE && mq_joint_foot_.size() > MIN_PO_QUEUE_SIZE && mq_foot_force_.size() > MIN_PO_QUEUE_SIZE) {
    return true;
  }
  return false;
}

// this function compares all the latest time of all the queues and returns
// the earliest one
double VILOFusion::getMIPOMinLatestTime() {
  if (mq_imu_.size() == 0 || mq_joint_foot_.size() == 0 || mq_fl_imu_.size() == 0 || mq_fr_imu_.size() == 0 || mq_rl_imu_.size() == 0 ||
      mq_rr_imu_.size() == 0) {
    return prev_esti_time;
  }
  double t_imu = mq_imu_.latestTime();
  double t_joint_foot = mq_joint_foot_.latestTime();
  double t_fl_imu = mq_fl_imu_.latestTime();
  double t_fr_imu = mq_fr_imu_.latestTime();
  double t_rl_imu = mq_rl_imu_.latestTime();
  double t_rr_imu = mq_rr_imu_.latestTime();

  double t = std::min(t_imu, t_joint_foot);
  t = std::min(t, t_fl_imu);
  t = std::min(t, t_fr_imu);
  t = std::min(t, t_rl_imu);
  t = std::min(t, t_rr_imu);

  return t;
}

double VILOFusion::getSIPOMinLatestTime() {
  if (mq_imu_.size() == 0 || mq_joint_foot_.size() == 0 || mq_foot_force_.size() == 0) {
    return prev_esti_time;
  }
  double t_imu = mq_imu_.latestTime();
  double t_joint_foot = mq_joint_foot_.latestTime();
  double t_foot_force = mq_foot_force_.latestTime();

  double t = std::min(t_imu, t_joint_foot);
  t = std::min(t, t_foot_force);

  return t;
}

double VILOFusion::getMIPOMaxOldestTime() {
  if (mq_imu_.size() == 0 || mq_joint_foot_.size() == 0 || mq_fl_imu_.size() == 0 || mq_fr_imu_.size() == 0 || mq_rl_imu_.size() == 0 ||
      mq_rr_imu_.size() == 0) {
    return prev_esti_time;
  }
  double t_imu = mq_imu_.oldestTime();
  double t_joint_foot = mq_joint_foot_.oldestTime();
  double t_fl_imu = mq_fl_imu_.oldestTime();
  double t_fr_imu = mq_fr_imu_.oldestTime();
  double t_rl_imu = mq_rl_imu_.oldestTime();
  double t_rr_imu = mq_rr_imu_.oldestTime();

  double t = std::max(t_imu, t_joint_foot);
  t = std::max(t, t_fl_imu);
  t = std::max(t, t_fr_imu);
  t = std::max(t, t_rl_imu);
  t = std::max(t, t_rr_imu);

  return t;
}

double VILOFusion::getSIPOMaxOldestTime() {
  if (mq_imu_.size() == 0 || mq_joint_foot_.size() == 0 || mq_foot_force_.size() == 0) {
    return prev_esti_time;
  }
  double t_imu = mq_imu_.oldestTime();
  double t_joint_foot = mq_joint_foot_.oldestTime();
  double t_foot_force = mq_foot_force_.oldestTime();

  double t = std::max(t_imu, t_joint_foot);
  t = std::max(t, t_foot_force);

  return t;
}

double VILOFusion::interpolateMIPOData(Eigen::Matrix<double, 55, 1>& sensor_data, double dt) {
  const std::lock_guard<std::mutex> lock(mtx);

  double queue_time_end = getMIPOMinLatestTime();
  double queue_time_start = getMIPOMaxOldestTime();

  double curr_esti_time = prev_esti_time + dt;
  if (curr_esti_time > queue_time_end) {
    // if the current time is larger than the queue time, we can use the
    // queue time to do the estimation
    curr_esti_time = queue_time_end;
  } else if (curr_esti_time < queue_time_start) {
    curr_esti_time = queue_time_start;
  }
  prev_esti_time = curr_esti_time;
  // now curr_esti_time must within the range of all the queues
  // we can use it to do interpolation
  std::shared_ptr<SWE::Measurement> imu_meas = mq_imu_.interpolate(curr_esti_time);
  std::shared_ptr<SWE::Measurement> joint_meas = mq_joint_foot_.interpolate(curr_esti_time);
  std::shared_ptr<SWE::Measurement> fl_imu_meas = mq_fl_imu_.interpolate(curr_esti_time);
  std::shared_ptr<SWE::Measurement> fr_imu_meas = mq_fr_imu_.interpolate(curr_esti_time);
  std::shared_ptr<SWE::Measurement> rl_imu_meas = mq_rl_imu_.interpolate(curr_esti_time);
  std::shared_ptr<SWE::Measurement> rr_imu_meas = mq_rr_imu_.interpolate(curr_esti_time);

  // input sensor data vectors to the estimator
  Eigen::VectorXd imu_data = imu_meas->getVector();
  Eigen::VectorXd joint_data = joint_meas->getVector();
  Eigen::VectorXd fl_imu_data = fl_imu_meas->getVector();
  Eigen::VectorXd fr_imu_data = fr_imu_meas->getVector();
  Eigen::VectorXd rl_imu_data = rl_imu_meas->getVector();
  Eigen::VectorXd rr_imu_data = rr_imu_meas->getVector();

  sensor_data.segment<3>(0) = imu_data.segment<3>(3);  // gyro
  sensor_data.segment<3>(3) = imu_data.segment<3>(0);  // acc, notice the order defined in
                                                       // MIPOEstimatorSensorData::loadFromVec
  sensor_data.segment<12>(6) = joint_data.segment<12>(0);
  sensor_data.segment<12>(18) = joint_data.segment<12>(12);
  sensor_data.segment<3>(30) = fl_imu_data.segment<3>(0);
  sensor_data.segment<3>(33) = fr_imu_data.segment<3>(0);
  sensor_data.segment<3>(36) = rl_imu_data.segment<3>(0);
  sensor_data.segment<3>(39) = rr_imu_data.segment<3>(0);
  sensor_data.segment<3>(42) = fl_imu_data.segment<3>(3) / 180.0 * M_PI;
  sensor_data.segment<3>(45) = fr_imu_data.segment<3>(3) / 180.0 * M_PI;
  sensor_data.segment<3>(48) = rl_imu_data.segment<3>(3) / 180.0 * M_PI;
  sensor_data.segment<3>(51) = rr_imu_data.segment<3>(3) / 180.0 * M_PI;

  sensor_data(54) = getYawObservation();

  return curr_esti_time;
}

double VILOFusion::interpolateSIPOData(Eigen::Matrix<double, 35, 1>& sensor_data, double dt) {
  const std::lock_guard<std::mutex> lock(mtx);

  double queue_time_end = getSIPOMinLatestTime();
  double queue_time_start = getSIPOMaxOldestTime();

  double curr_esti_time = prev_esti_time + dt;
  if (curr_esti_time > queue_time_end) {
    // if the current time is larger than the queue time, we can use the
    // queue time to do the estimation
    curr_esti_time = queue_time_end;
  } else if (curr_esti_time < queue_time_start) {
    curr_esti_time = queue_time_start;
  }
  prev_esti_time = curr_esti_time;
  // now curr_esti_time must within the range of all the queues
  // we can use it to do interpolation
  std::shared_ptr<SWE::Measurement> imu_meas = mq_imu_.interpolate(curr_esti_time);
  std::shared_ptr<SWE::Measurement> joint_meas = mq_joint_foot_.interpolate(curr_esti_time);
  std::shared_ptr<SWE::Measurement> force_meas = mq_foot_force_.interpolate(curr_esti_time);

  // input sensor data vectors to the estimator
  Eigen::VectorXd imu_data = imu_meas->getVector();
  Eigen::VectorXd joint_data = joint_meas->getVector();
  Eigen::VectorXd force_data = force_meas->getVector();

  sensor_data.segment<3>(0) = imu_data.segment<3>(3);  // gyro
  sensor_data.segment<3>(3) = imu_data.segment<3>(0);  // acc, notice the order defined in
                                                       // SIPOEstimatorSensorData::loadFromVec
  sensor_data.segment<12>(6) = joint_data.segment<12>(0);
  sensor_data.segment<12>(18) = joint_data.segment<12>(12);
  sensor_data.segment<4>(30) = force_data.segment<4>(0);

  sensor_data(34) = getYawObservation();

  return curr_esti_time;
}

double VILOFusion::getYawObservation() {
  // source of yaw observation
  double yaw_observation;
  double yaw_source = 0.0;
  if (!vilo_estimator->isRunning()) {  // vilo not started yet
    // timing of ground truth is not very critical we just use the latest
    if (mq_gt_.size() > 0) {
      std::shared_ptr<SWE::Measurement> gt_meas = mq_gt_.top();
      latest_gt_meas = gt_meas;

      Eigen::VectorXd gt_data = latest_gt_meas->getVector();
      Eigen::VectorXd gt_quat_vec = gt_data.segment<4>(3);
      Eigen::Quaterniond gt_quat(gt_quat_vec(0), gt_quat_vec(1), gt_quat_vec(2), gt_quat_vec(3));
      Eigen::Vector3d gt_euler = legged::quat_to_euler<double>(gt_quat);
      // item 54 is the body yaw, set to 0 for now
      yaw_source = gt_euler(2);
    } else {
      // gt is not available, then just treat the yaw as 0
      yaw_source = 0.0;
    }
  } else {
    // use the yaw output from the VILO estimator
    // yaw is observable anyway so timing is not critical
    Eigen::VectorXd x_vilo = vilo_estimator->outputState();
    Eigen::VectorXd vilo_quat_vec = x_vilo.segment<4>(4);  // x y z w
    Eigen::Quaterniond quat(vilo_quat_vec(3), vilo_quat_vec(0), vilo_quat_vec(1), vilo_quat_vec(2));
    Eigen::Vector3d euler = legged::quat_to_euler<double>(quat);
    yaw_source = euler(2);
  }
  // wrap the yaw_source to make it continuous
  if (prev_po_input_yaw > M_PI * 0.8 && yaw_source < prev_po_input_yaw - M_PI) {
    yaw_source += 2.0 * M_PI;
  } else if (prev_po_input_yaw < -M_PI * 0.8 && yaw_source > prev_po_input_yaw + M_PI) {
    yaw_source -= 2.0 * M_PI;
  }

  yaw_observation = yaw_filter_.CalculateAverage(yaw_source);
  prev_po_input_yaw = yaw_observation;

  return yaw_observation;
}