
#include "utils/ROSFusion.hpp"
#include "utils/casadi_kino.hpp"

ROSFusion::ROSFusion(ros::NodeHandle nh) {
  nh_ = nh;
  readParameters();

  // bind the loop_thread_ to the loop function
  loop_thread_ = std::thread(&ROSFusion::loop, this);

  // initialize callback functions
  imu_sub_ = nh_.subscribe(IMU_TOPIC, 1000, &ROSFusion::imuCallback, this);
  joint_foot_sub_ = nh_.subscribe(JOINT_FOOT_TOPIC, 1000,
                                  &ROSFusion::jointFootCallback, this);
  fl_imu_sub_ =
      nh_.subscribe(FL_IMU_TOPIC, 1000, &ROSFusion::flImuCallback, this);
  fr_imu_sub_ =
      nh_.subscribe(FR_IMU_TOPIC, 1000, &ROSFusion::frImuCallback, this);
  rl_imu_sub_ =
      nh_.subscribe(RL_IMU_TOPIC, 1000, &ROSFusion::rlImuCallback, this);
  rr_imu_sub_ =
      nh_.subscribe(RR_IMU_TOPIC, 1000, &ROSFusion::rrImuCallback, this);
  gt_sub_ = nh_.subscribe(GT_TOPIC, 1000, &ROSFusion::gtCallback, this);

  // initialize queues and specify their main types
  mq_imu_ = SWE::MeasureQueue(SWE::MeasureType::BODY_IMU);
  mq_joint_foot_ = SWE::MeasureQueue(SWE::MeasureType::LEG);
  mq_fl_imu_ = SWE::MeasureQueue(SWE::MeasureType::BODY_IMU);
  mq_fr_imu_ = SWE::MeasureQueue(SWE::MeasureType::BODY_IMU);
  mq_rl_imu_ = SWE::MeasureQueue(SWE::MeasureType::BODY_IMU);
  mq_rr_imu_ = SWE::MeasureQueue(SWE::MeasureType::BODY_IMU);
  mq_gt_ = SWE::MeasureQueue(SWE::MeasureType::DIRECT_POSE);

  // initialize publishers
  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "/mipo/estimate_pose", 1000);

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
}

void ROSFusion::readParameters() {
  // parameters default values
  IMU_TOPIC = "/unitree_hardware/imu";
  JOINT_FOOT_TOPIC = "/unitree_hardware/joint_foot";
  FL_IMU_TOPIC = "/WT901_49_Data";
  FR_IMU_TOPIC = "/WT901_48_Data";
  RL_IMU_TOPIC = "/WT901_50_Data";
  RR_IMU_TOPIC = "/WT901_47_Data";
  GT_TOPIC = "/mocap_node/Go1_body/pose";

  // read parameters from ros server
  nh_.param<std::string>("IMU_TOPIC", IMU_TOPIC, "/unitree_hardware/imu");
  nh_.param<std::string>("JOINT_FOOT_TOPIC", JOINT_FOOT_TOPIC,
                         "/unitree_hardware/joint_foot");
  nh_.param<std::string>("FL_IMU_TOPIC", FL_IMU_TOPIC, "/WT901_49_Data");
  nh_.param<std::string>("FR_IMU_TOPIC", FR_IMU_TOPIC, "/WT901_48_Data");
  nh_.param<std::string>("RL_IMU_TOPIC", RL_IMU_TOPIC, "/WT901_50_Data");
  nh_.param<std::string>("RR_IMU_TOPIC", RR_IMU_TOPIC, "/WT901_47_Data");
  nh_.param<std::string>("GT_TOPIC", GT_TOPIC, "/mocap_node/Go1_body/pose");
}

// constant loop
// we control the time using ros time,
// but we also calculate the loop time using chrono so we know how long the EKF
// update takes
void ROSFusion::loop() {
  const double LOOP_DT = 5.0; // 5ms
  prev_loop_time = ros::Time::now().toSec();
  prev_esti_time = ros::Time::now().toSec();

  std::shared_ptr<MIPOEstimatorSensorData> prev_data = nullptr;
  std::shared_ptr<MIPOEstimatorSensorData> curr_data = nullptr;

  // initialize the estimator
  MIPOEstimator mipo_estimator;
  Eigen::Matrix<double, MS_SIZE, 1> x;       // state
  Eigen::Matrix<double, MS_SIZE, MS_SIZE> P; // covariance

  while (1) {
    /* record start time */
    auto loop_start = std::chrono::system_clock::now();
    auto ros_loop_start = ros::Time::now().toSec();

    /* get a bunch of ros objects */
    double curr_loop_time = ros::Time::now().toSec();
    double dt_ros = curr_loop_time - prev_loop_time;
    prev_loop_time = curr_loop_time;
    std::cout << "dt_ros: " << dt_ros << std::endl;
    if (dt_ros == 0) {
      continue;
    }

    /* estimator logic */
    if (isDataAvailable()) {
      mtx.lock();
      // first, given time stamps of all queues, get a time t which is a
      // little earlier than all the measurements in the queues then,
      // interpolate all the measurements at time t finally, push the
      // interpolated measurements into the estimator
      double queue_time_end = getMinLatestTime();
      double queue_time_start = getMaxOldestTime();

      std::cout << "stage 0" << std::endl;
      double curr_esti_time = prev_esti_time + dt_ros;
      if (curr_esti_time > queue_time_end) {
        // if the current time is larger than the queue time, we can use the
        // queue time to do the estimation
        curr_esti_time = queue_time_end;
      } else if (curr_esti_time < queue_time_start) {
        curr_esti_time = queue_time_start;
      }
      prev_esti_time = curr_esti_time;

      std::cout << "queue_time_end t: " << std::setprecision(15)
                << queue_time_end << std::endl;
      std::cout << "curr_esti_time t: " << std::setprecision(15)
                << curr_esti_time << std::endl;
      // now curr_esti_time must within the range of all the queues
      // we can use it to do interpolation
      std::shared_ptr<SWE::Measurement> imu_meas =
          mq_imu_.interpolate(curr_esti_time);
      std::shared_ptr<SWE::Measurement> joint_meas =
          mq_joint_foot_.interpolate(curr_esti_time);
      std::shared_ptr<SWE::Measurement> fl_imu_meas =
          mq_fl_imu_.interpolate(curr_esti_time);
      std::shared_ptr<SWE::Measurement> fr_imu_meas =
          mq_fr_imu_.interpolate(curr_esti_time);
      std::shared_ptr<SWE::Measurement> rl_imu_meas =
          mq_rl_imu_.interpolate(curr_esti_time);
      std::shared_ptr<SWE::Measurement> rr_imu_meas =
          mq_rr_imu_.interpolate(curr_esti_time);

      // timing of ground truth is not very critical we just use the latest
      std::shared_ptr<SWE::Measurement> gt_meas = mq_gt_.top();
      if (gt_meas != nullptr) {
        latest_gt_meas = gt_meas;
      }

      std::cout << "stage 1" << std::endl;
      // input sensor data vectors to the estimator
      Eigen::VectorXd imu_data = imu_meas->getVector();
      Eigen::VectorXd joint_data = joint_meas->getVector();
      Eigen::VectorXd fl_imu_data = fl_imu_meas->getVector();
      Eigen::VectorXd fr_imu_data = fr_imu_meas->getVector();
      Eigen::VectorXd rl_imu_data = rl_imu_meas->getVector();
      Eigen::VectorXd rr_imu_data = rr_imu_meas->getVector();
      Eigen::VectorXd gt_data = latest_gt_meas->getVector();
      Eigen::VectorXd gt_quat_vec = gt_data.segment<4>(3);
      Eigen::Quaterniond gt_quat(gt_quat_vec(0), gt_quat_vec(1), gt_quat_vec(2),
                                 gt_quat_vec(3));
      Eigen::Vector3d gt_euler = legged::quat_to_euler<double>(gt_quat);
      // print out the data to see if they are correct
      // std::cout << "imu_data: " << imu_data.transpose() << std::endl;
      // std::cout << "joint_data: " << joint_data.transpose() << std::endl;
      // std::cout << "fl_imu_data: " << fl_imu_data.transpose() <<
      // std::endl; std::cout << "fr_imu_data: " << fr_imu_data.transpose()
      // << std::endl; std::cout << "rl_imu_data: " <<
      // rl_imu_data.transpose() << std::endl; std::cout << "rr_imu_data: "
      // << rr_imu_data.transpose() << std::endl;

      std::cout << "stage 2" << std::endl;
      Eigen::Matrix<double, 55, 1> sensor_data;
      sensor_data.segment<3>(0) = imu_data.segment<3>(3); // gyro
      sensor_data.segment<3>(3) =
          imu_data.segment<3>(0); // acc, notice the order defined in
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

      // item 54 is the body yaw, set to 0 for now
      sensor_data(54) = gt_euler(2);

      // std::cout << "sensor_data: " << sensor_data.transpose() << std::endl;

      mtx.unlock();

      std::cout << "stage 3" << std::endl;
      // get estimation result from the estimator, publish them as ros topics
      // save sensor data to prev_data and curr_data

      if (prev_data == nullptr) {
        prev_data = std::make_shared<MIPOEstimatorSensorData>();
        prev_data->loadFromVec(sensor_data);
        // initialize the estimator
        x = mipo_estimator.ekfInitState(*prev_data);
        P = 1e-4 * Eigen::Matrix<double, MS_SIZE, MS_SIZE>::Identity();
      } else {
        curr_data = std::make_shared<MIPOEstimatorSensorData>();
        curr_data->loadFromVec(sensor_data);

        // do estimation
        Eigen::Matrix<double, MS_SIZE, 1> x_k1_est;
        Eigen::Matrix<double, MS_SIZE, MS_SIZE> P_k1_est;
        mipo_estimator.ekfUpdate(x, P, *prev_data, *curr_data, dt_ros, x_k1_est,
                                 P_k1_est);

        x = x_k1_est;
        // std::cout << "x: " << x.transpose() << std::endl;
        P = P_k1_est;
        prev_data = curr_data;
        // publish the estimation result
        publishEstimationResult(x, P, curr_esti_time);
      }
      std::cout << "stage 4" << std::endl;

      // TODO: save ground truth data and the estimation result to compare
      // if (is_gt_available_) {

      //   std::shared_ptr<SWE::Measurement> gt_meas =
      //       mq_gt_.interpolate(curr_esti_time);
      //   Eigen::VectorXd gt_data = gt_meas->getVector();
      // }

    } else {
      prev_esti_time += dt_ros;
    }

    /* record end time */
    auto loop_end = std::chrono::system_clock::now();
    auto loop_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
        loop_end - loop_start);
    std::cout << "EKF solve time: " << loop_elapsed.count() << std::endl;

    auto ros_loop_end = ros::Time::now().toSec();
    auto ros_loop_elapsed = ros_loop_end - ros_loop_start;

    /* sleep a while to make sure the loop run at LOOP_DT */
    if (ros_loop_elapsed * 1000 >= LOOP_DT) {
      // do not sleep
      std::cout << "loop computation time is longer than desired dt, optimize "
                   "code or increase LOOP_DT"
                << std::endl;
    } else {
      double sleep_time = LOOP_DT * 0.001 - ros_loop_elapsed;
      ros::Duration(sleep_time).sleep();
    }
    /* check total loop actual CPU time */
    // auto total_loop_end = std::chrono::system_clock::now();
    // std::chrono::duration<double,std::milli> total_loop_elapsed =
    // total_loop_end - loop_start; std::cout << std::setw(9) << "Total loop
    // time " << total_loop_elapsed.count() << " ms"<< std::endl;
  }
  return;
}

//
bool ROSFusion::isDataAvailable() {
  if (mq_imu_.size() > MIN_WINDOW_SIZE &&
      mq_joint_foot_.size() > MIN_WINDOW_SIZE &&
      mq_fl_imu_.size() > MIN_WINDOW_SIZE &&
      mq_fr_imu_.size() > MIN_WINDOW_SIZE &&
      mq_rl_imu_.size() > MIN_WINDOW_SIZE &&
      mq_rr_imu_.size() > MIN_WINDOW_SIZE && mq_gt_.size() > MIN_WINDOW_SIZE) {
    return true;
  }
  return false;
}
// this function compares all the latest time of all the queues and returns the
// earliest one
double ROSFusion::getMinLatestTime() {
  if (mq_imu_.size() == 0 || mq_joint_foot_.size() == 0 ||
      mq_fl_imu_.size() == 0 || mq_fr_imu_.size() == 0 ||
      mq_rl_imu_.size() == 0 || mq_rr_imu_.size() == 0) {
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

  // if have gt, also consider gt time
  // if (is_gt_available_) {
  //   if (mq_gt_.size() > 0) {
  //     double t_gt = mq_gt_.latestTime();
  //     t = std::min(t, t_gt);
  //   }
  // }

  return t;
}

double ROSFusion::getMaxOldestTime() {
  if (mq_imu_.size() == 0 || mq_joint_foot_.size() == 0 ||
      mq_fl_imu_.size() == 0 || mq_fr_imu_.size() == 0 ||
      mq_rl_imu_.size() == 0 || mq_rr_imu_.size() == 0) {
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

  // if have gt, also consider gt time
  // if (is_gt_available_) {
  //   if (mq_gt_.size() > 0) {
  //     double t_gt = mq_gt_.oldestTime();
  //     t = std::min(t, t_gt);
  //   }
  // }

  return t;
}

void ROSFusion::publishEstimationResult(
    Eigen::Matrix<double, MS_SIZE, 1> &x,
    Eigen::Matrix<double, MS_SIZE, MS_SIZE> &P, double cur_time) {
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
  pose_msg.header.stamp = ros::Time(cur_time);
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

  return;
}

// callback functions
void ROSFusion::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  double t = msg->header.stamp.toSec();
  // assemble sensor data
  Eigen::Vector3d acc =
      Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y,
                      msg->linear_acceleration.z);
  Eigen::Vector3d ang_vel =
      Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                      msg->angular_velocity.z);

  mtx.lock();
  mq_imu_.push(t, acc, ang_vel);

  // limit the size of the message queue
  if (!mq_imu_.empty() && mq_imu_.size() > MAX_MESSAGE_QUEUE_SIZE) {
    mq_imu_.pop();
  }
  mtx.unlock();

  return;
}

void ROSFusion::jointFootCallback(
    const sensor_msgs::JointState::ConstPtr &msg) {
  double t = msg->header.stamp.toSec();
  Eigen::Matrix<double, 12, 1> joint_pos;
  Eigen::Matrix<double, 12, 1> joint_vel;

  for (int i = 0; i < NUM_DOF; i++) {
    joint_pos(i) = msg->position[i];
    joint_vel(i) = joint_foot_filter_[i].CalculateAverage(msg->velocity[i]);
  }

  mtx.lock();
  mq_joint_foot_.push(t, joint_pos, joint_vel);
  // limit the size of the message queue
  if (!mq_joint_foot_.empty() &&
      mq_joint_foot_.size() > MAX_MESSAGE_QUEUE_SIZE) {
    mq_joint_foot_.pop();
  }
  mtx.unlock();

  return;
}

void ROSFusion::flImuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  double t = msg->header.stamp.toSec() - FOOT_IMU_DELAY;
  // assemble sensor data
  Eigen::Vector3d acc = Eigen::Vector3d(
      fl_imu_acc_filter_[0].CalculateAverage(msg->linear_acceleration.x),
      fl_imu_acc_filter_[1].CalculateAverage(msg->linear_acceleration.y),
      fl_imu_acc_filter_[2].CalculateAverage(msg->linear_acceleration.z));
  Eigen::Vector3d ang_vel = Eigen::Vector3d(
      fl_imu_gyro_filter_[0].CalculateAverage(msg->angular_velocity.x),
      fl_imu_gyro_filter_[1].CalculateAverage(msg->angular_velocity.y),
      fl_imu_gyro_filter_[2].CalculateAverage(msg->angular_velocity.z));

  mtx.lock();
  mq_fl_imu_.push(t, acc, ang_vel, 0);
  // limit the size of the message queue
  if (!mq_fl_imu_.empty() && mq_fl_imu_.size() > MAX_MESSAGE_QUEUE_SIZE) {
    mq_fl_imu_.pop();
  }
  mtx.unlock();

  return;
}

void ROSFusion::frImuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  double t = msg->header.stamp.toSec() - FOOT_IMU_DELAY;
  // assemble sensor data
  Eigen::Vector3d acc = Eigen::Vector3d(
      fr_imu_acc_filter_[0].CalculateAverage(msg->linear_acceleration.x),
      fr_imu_acc_filter_[1].CalculateAverage(msg->linear_acceleration.y),
      fr_imu_acc_filter_[2].CalculateAverage(msg->linear_acceleration.z));
  Eigen::Vector3d ang_vel = Eigen::Vector3d(
      fr_imu_gyro_filter_[0].CalculateAverage(msg->angular_velocity.x),
      fr_imu_gyro_filter_[1].CalculateAverage(msg->angular_velocity.y),
      fr_imu_gyro_filter_[2].CalculateAverage(msg->angular_velocity.z));

  mtx.lock();
  mq_fr_imu_.push(t, acc, ang_vel, 1);
  // limit the size of the message queue
  if (!mq_fr_imu_.empty() && mq_fr_imu_.size() > MAX_MESSAGE_QUEUE_SIZE) {
    mq_fr_imu_.pop();
  }
  mtx.unlock();

  return;
}

void ROSFusion::rlImuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  double t = msg->header.stamp.toSec() - FOOT_IMU_DELAY;
  // assemble sensor data
  Eigen::Vector3d acc = Eigen::Vector3d(
      rl_imu_acc_filter_[0].CalculateAverage(msg->linear_acceleration.x),
      rl_imu_acc_filter_[1].CalculateAverage(msg->linear_acceleration.y),
      rl_imu_acc_filter_[2].CalculateAverage(msg->linear_acceleration.z));
  Eigen::Vector3d ang_vel = Eigen::Vector3d(
      rl_imu_gyro_filter_[0].CalculateAverage(msg->angular_velocity.x),
      rl_imu_gyro_filter_[1].CalculateAverage(msg->angular_velocity.y),
      rl_imu_gyro_filter_[2].CalculateAverage(msg->angular_velocity.z));

  mtx.lock();
  mq_rl_imu_.push(t, acc, ang_vel, 2);
  // limit the size of the message queue
  if (!mq_rl_imu_.empty() && mq_rl_imu_.size() > MAX_MESSAGE_QUEUE_SIZE) {
    mq_rl_imu_.pop();
  }
  mtx.unlock();

  return;
}

void ROSFusion::rrImuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  double t = msg->header.stamp.toSec() - FOOT_IMU_DELAY;
  // assemble sensor data
  Eigen::Vector3d acc = Eigen::Vector3d(
      rr_imu_acc_filter_[0].CalculateAverage(msg->linear_acceleration.x),
      rr_imu_acc_filter_[1].CalculateAverage(msg->linear_acceleration.y),
      rr_imu_acc_filter_[2].CalculateAverage(msg->linear_acceleration.z));
  Eigen::Vector3d ang_vel = Eigen::Vector3d(
      rr_imu_gyro_filter_[0].CalculateAverage(msg->angular_velocity.x),
      rr_imu_gyro_filter_[1].CalculateAverage(msg->angular_velocity.y),
      rr_imu_gyro_filter_[2].CalculateAverage(msg->angular_velocity.z));

  mtx.lock();
  mq_rr_imu_.push(t, acc, ang_vel, 3);
  // limit the size of the message queue
  if (!mq_rr_imu_.empty() && mq_rr_imu_.size() > MAX_MESSAGE_QUEUE_SIZE) {
    mq_rr_imu_.pop();
  }
  mtx.unlock();

  return;
}

// also receive and record the ground truth for training/debuging purpose
void ROSFusion::gtCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  double t = msg->header.stamp.toSec();
  // get position and orientation
  Eigen::Vector3d pos = Eigen::Vector3d(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                       msg->pose.orientation.y, msg->pose.orientation.z);

  mq_gt_.push(t, pos, q);
  // limit the size of the message queue
  if (!mq_gt_.empty() && mq_gt_.size() > MAX_MESSAGE_QUEUE_SIZE) {
    mq_gt_.pop();
  }

  // if we have received the ground truth, we set the flag to true
  is_gt_available_ = true;
  return;
}
