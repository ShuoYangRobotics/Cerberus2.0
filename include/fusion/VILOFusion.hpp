#pragma once
// standard C library
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <queue>

// opencv and eigen, eigen must go first
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

// include ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>

// project files
#include "utils/Measurement.hpp"
#include "utils/MeasurementQueue.hpp"
#include "utils/MovingWindowFilter.hpp"
#include "utils/parameters.hpp"

// this file is similar to ROSFusion.hpp but it also fuses camera data
// the process of vision part is similar to cerberus' main.cpp

class VILOFusion {
public:
  VILOFusion(ros::NodeHandle nh);
  ~VILOFusion(){};


  void loop();
  bool isDataAvailable();
  double getMinLatestTime();
  double getMaxOldestTime();

  void publishEstimationResult(Eigen::VectorXd &x,
                               Eigen::MatrixXd &P,
                               double cur_time);

  // initialize callback functions
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void jointFootCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void flImuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void frImuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void rlImuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void rrImuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void gtCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

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


  std::thread loop_thread_;

  // measurement queues filled by callback functions
  SWE::MeasureQueue mq_imu_;
  SWE::MeasureQueue mq_joint_foot_;
  SWE::MeasureQueue mq_fl_imu_;
  SWE::MeasureQueue mq_fr_imu_;
  SWE::MeasureQueue mq_rl_imu_;
  SWE::MeasureQueue mq_rr_imu_;
  SWE::MeasureQueue mq_gt_;
  bool is_gt_available_; // maybe gt is not available, we need to check this
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

  // private helper functions
  void img0_callback(const sensor_msgs::ImageConstPtr& img_msg);
  void img1_callback(const sensor_msgs::ImageConstPtr& img_msg);
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr& img_msg);

  void inputImages();
};