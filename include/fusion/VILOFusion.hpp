#pragma once
// standard C library
#include <iomanip>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

// opencv and eigen, eigen must go first
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

// include ros
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>

// project files
#include "mipo/MIPOEstimator.hpp"
#include "sipo/SIPOEstimator.hpp"
#include "utils/Measurement.hpp"
#include "utils/MeasurementQueue.hpp"
#include "utils/MovingWindowFilter.hpp"
#include "utils/casadi_kino.hpp"
#include "utils/parameters.hpp"
#include "vilo/VILOEstimator.hpp"

// this file is similar to ROSFusion.hpp but it also fuses camera data
// the process of vision part is similar to cerberus' main.cpp

class VILOFusion {
 public:
  VILOFusion(ros::NodeHandle nh);
  ~VILOFusion();

  void POLoop();    // this loop runs MIPO at 400Hz
  void VILOLoop();  // this loop triggers VILO at 15Hz

  void publishPOEstimationResult(Eigen::VectorXd& x, Eigen::MatrixXd& P);

  void publishVILOEstimationResult(Eigen::Matrix<double, VS_OUTSIZE, 1>& state);

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

  ros::Subscriber img0_sub_;
  ros::Subscriber img1_sub_;

  // ros publishers for debug
  ros::Publisher pose_pub_;        // MIPO pose
  ros::Publisher pose_vilo_pub_;   // VILO pose
  ros::Publisher twist_pub_;       // MIPO twist
  ros::Publisher twist_vilo_pub_;  // VILO twist

  std::thread po_loop_thread_;
  std::thread vilo_loop_thread_;

  // filters
  std::unique_ptr<MIPOEstimator> mipo_estimator;
  std::unique_ptr<SIPOEstimator> sipo_estimator;
  std::unique_ptr<VILOEstimator> vilo_estimator;

  // measurement queues filled by callback functions
  SWE::MeasureQueue mq_imu_;
  SWE::MeasureQueue mq_joint_foot_;
  SWE::MeasureQueue mq_foot_force_;
  SWE::MeasureQueue mq_fl_imu_;
  SWE::MeasureQueue mq_fr_imu_;
  SWE::MeasureQueue mq_rl_imu_;
  SWE::MeasureQueue mq_rr_imu_;
  SWE::MeasureQueue mq_gt_;
  bool is_gt_available_;  // maybe gt is not available, we need to check this
  std::shared_ptr<SWE::Measurement> latest_gt_meas;
  // foot imu filter
  MovingWindowFilter joint_foot_filter_[12];
  MovingWindowFilter fl_imu_acc_filter_[3];
  MovingWindowFilter fr_imu_acc_filter_[3];
  MovingWindowFilter rl_imu_acc_filter_[3];
  MovingWindowFilter rr_imu_acc_filter_[3];
  MovingWindowFilter fl_imu_gyro_filter_[3];
  MovingWindowFilter fr_imu_gyro_filter_[3];
  MovingWindowFilter rl_imu_gyro_filter_[3];
  MovingWindowFilter rr_imu_gyro_filter_[3];

  // yaw observation processing
  double prev_po_input_yaw;
  MovingWindowFilter yaw_filter_;

  // prev loop time
  double prev_loop_time;
  double prev_esti_time;

  // mutex to protect the queue from been read and written at the same time
  std::mutex mtx;

  // stereo vision related
  std::queue<sensor_msgs::PointCloudConstPtr> feature_buf;
  std::queue<sensor_msgs::ImageConstPtr> img0_buf;
  std::queue<sensor_msgs::ImageConstPtr> img1_buf;
  std::mutex mtx_image;

  // save results
  Eigen::Matrix<double, MS_SIZE, 1> mipo_x;        // MIPO state
  Eigen::Matrix<double, MS_SIZE, MS_SIZE> mipo_P;  // MIPO covariance
  Eigen::Matrix<double, SS_SIZE, 1> sipo_x;        // SIPO state
  Eigen::Matrix<double, SS_SIZE, SS_SIZE> sipo_P;  // SIPO covariance

  Eigen::VectorXd po_x;  // PO state for recording

  Eigen::Matrix<double, 17, 1> vilo_x;  // VILO state

  // private helper functions
  void img0Callback(const sensor_msgs::ImageConstPtr& img_msg);
  void img1Callback(const sensor_msgs::ImageConstPtr& img_msg);
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr& img_msg);

  void inputImagesToVILO();

  // initialize proprioceptive sensor callback functions
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void jointFootCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void flImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void frImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void rlImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void rrImuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  // initialize gt callback
  void gtCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  bool isMIPODataAvailable();
  double getMIPOMinLatestTime();
  double getMIPOMaxOldestTime();
  double interpolateMIPOData(Eigen::Matrix<double, 55, 1>& sensor_data, double dt);

  bool isSIPODataAvailable();
  double getSIPOMinLatestTime();
  double getSIPOMaxOldestTime();
  double interpolateSIPOData(Eigen::Matrix<double, 35, 1>& sensor_data, double dt);

  // calculate a yaw observation from gt or vilo
  double getYawObservation();
};