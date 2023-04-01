#pragma once
// standard C library
#include <Eigen/Dense>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
// include ros
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

// measurement queues to handle measurement time delay and frequency mismatch
#include "mipo/MIPOEstimator.hpp"
#include "utils/Measurement.hpp"
#include "utils/MeasurementQueue.hpp"
#include "utils/parameters.hpp"

// this is a helper class to bridge ros messages and robot estimator
// the class instatiates ros handle, rostopic subscriber and publisher, as well
// as the robot estimator so that the robot estimator is ros free

class ROSFusion {
 public:
  ROSFusion(ros::NodeHandle nh);
  ~ROSFusion(){};

  void readParameters();

  void loop();
  bool isDataAvailable();
  double getOldestLatestTime();

  void publishEstimationResult(Eigen::Matrix<double, MS_SIZE, 1>& x, Eigen::Matrix<double, MS_SIZE, MS_SIZE>& P, double cur_time);

  // initialize callback functions
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void jointFootCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void flImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void frImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void rlImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void rrImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void gtCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

 private:
  // ros handle
  ros::NodeHandle nh_;

  // ros subscribers
  ros::Subscriber imu_sub_;
  ros::Subscriber joint_foot_sub_;
  ros::Subscriber fl_imu_sub_;
  ros::Subscriber fr_imu_sub_;
  ros::Subscriber rl_imu_sub_;
  ros::Subscriber rr_imu_sub_;
  ros::Subscriber gt_sub_;

  // ros publishers for debug
  ros::Publisher pose_pub_;

  // topic names
  std::string IMU_TOPIC;
  std::string JOINT_FOOT_TOPIC;
  std::string FL_IMU_TOPIC;
  std::string FR_IMU_TOPIC;
  std::string RL_IMU_TOPIC;
  std::string RR_IMU_TOPIC;
  std::string GT_TOPIC;

  std::thread loop_thread_;

  // measurement queues filled by callback functions
  SWE::MeasureQueue mq_imu_;
  SWE::MeasureQueue mq_joint_foot_;
  SWE::MeasureQueue mq_fl_imu_;
  SWE::MeasureQueue mq_fr_imu_;
  SWE::MeasureQueue mq_rl_imu_;
  SWE::MeasureQueue mq_rr_imu_;
  SWE::MeasureQueue mq_gt_;
  double foot_delay = FOOT_IMU_DELAY;
  bool is_gt_available_;  // maybe gt is not available, we need to check this
  // init gt position
  Eigen::Vector3d init_gt_pos;

  // prev loop time
  double prev_loop_time;
  double prev_esti_time;

  // mutex to protect the queue from been read and written at the same time
  std::mutex mtx;
};