#pragma once

#include <Eigen/Dense>
#include <memory>

namespace SWE {
enum MeasureType {
  BODY_IMU, // might further decompose this into BODY_ACC and BODY_GYRO
  LEG,
  FOOT_IMU,
  FOOT_FORCE,
  DIRECT_POSE
};

// define abstract measurement type
// https://stackoverflow.com/questions/19678011/c-multiple-type-array
// time is the only common variable
class Measurement {
public:
  Measurement() {}
  virtual MeasureType getType() = 0;
  virtual double getTime() = 0;
  virtual Eigen::VectorXd getVector() = 0;
};

// each measurement data struct has
//    t   - the timestamp of the sensor package
//   type - MeasureType
//   actual data that depepnds on measurement type

// measurement from the robot hardaware IMU or gazebo sim
class BodyIMUMeasurement : public Measurement {
public:
  BodyIMUMeasurement() {}

  BodyIMUMeasurement(double _t, Eigen::Vector3d _imu_acc,
                     Eigen::Vector3d _imu_gyro) {
    t = _t;
    imu_acc = _imu_acc;
    imu_gyro = _imu_gyro;
  }
  MeasureType getType() { return type; }
  double getTime() { return t; }
  Eigen::VectorXd getVector() {
    Eigen::VectorXd v(6);
    v << imu_acc, imu_gyro;
    return v;
  }

  // helper function, return a interpolated measurement between two measurements
  // for brevity we assume t1 < t < t2, caller should make sure this is true
  // return a smart pointer to a BodyIMUMeasurement so if no one is using it, it
  // will be deleted authomatically
  static std::shared_ptr<BodyIMUMeasurement>
  interpolate(std::shared_ptr<BodyIMUMeasurement> m1,
              std::shared_ptr<BodyIMUMeasurement> m2, double t) {
    double t1 = m1->t;
    double t2 = m2->t;
    double alpha = (t - t1) / (t2 - t1);
    Eigen::Vector3d imu_acc = m1->imu_acc * (1 - alpha) + m2->imu_acc * alpha;
    Eigen::Vector3d imu_gyro =
        m1->imu_gyro * (1 - alpha) + m2->imu_gyro * alpha;
    return std::make_shared<BodyIMUMeasurement>(t, imu_acc, imu_gyro);
  }

  Eigen::Vector3d imu_acc;
  Eigen::Vector3d imu_gyro;

  MeasureType type = BODY_IMU;
  double t;
};

// measurement from the robot hardware joint encoders or gazebo sim
// notice we do not assume each leg is individual
class LegMeasurement : public Measurement {
public:
  LegMeasurement() {}

  LegMeasurement(double _t, Eigen::Matrix<double, 12, 1> _joint_pos,
                 Eigen::Matrix<double, 12, 1> _joint_vel) {
    t = _t;
    joint_pos = _joint_pos;
    joint_vel = _joint_vel;
  }

  LegMeasurement(double _t, Eigen::Matrix<double, 12, 1> _joint_pos,
                 Eigen::Matrix<double, 12, 1> _joint_vel,
                 Eigen::Matrix<double, 12, 1> _joint_tau) {
    t = _t;
    joint_pos = _joint_pos;
    joint_vel = _joint_vel;
    joint_tau = _joint_tau;
  }
  MeasureType getType() { return type; }
  double getTime() { return t; }
  Eigen::VectorXd getVector() {
    Eigen::VectorXd v(36);
    v << joint_pos, joint_vel, joint_tau;
    return v;
  }

  // helper function, return a interpolated measurement between two measurements
  // for brevity we assume t1 < t < t2, caller should make sure this is true
  static std::shared_ptr<LegMeasurement>
  interpolate(std::shared_ptr<LegMeasurement> m1,
              std::shared_ptr<LegMeasurement> m2, double t) {
    double t1 = m1->t;
    double t2 = m2->t;
    double alpha = (t - t1) / (t2 - t1);
    Eigen::Matrix<double, 12, 1> joint_pos =
        m1->joint_pos * (1 - alpha) + m2->joint_pos * alpha;
    Eigen::Matrix<double, 12, 1> joint_vel =
        m1->joint_vel * (1 - alpha) + m2->joint_vel * alpha;
    Eigen::Matrix<double, 12, 1> joint_tau =
        m1->joint_tau * (1 - alpha) + m2->joint_tau * alpha;
    return std::make_shared<LegMeasurement>(t, joint_pos, joint_vel, joint_tau);
  }

  Eigen::Matrix<double, 12, 1> joint_pos;
  Eigen::Matrix<double, 12, 1> joint_vel;
  Eigen::Matrix<double, 12, 1> joint_tau;

  MeasureType type = LEG;
  double t;
};

// measurement from the foot IMUs
// notice we DO assume each leg is an individual IMU package
class FootIMUMeasurement : public Measurement {
public:
  FootIMUMeasurement() {}

  FootIMUMeasurement(double _t, Eigen::Vector3d _imu_acc,
                     Eigen::Vector3d _imu_gyro, int _id) {
    t = _t;
    imu_acc = _imu_acc;
    imu_gyro = _imu_gyro;
    id = _id;
  }
  MeasureType getType() { return type; }
  double getTime() { return t; }
  Eigen::VectorXd getVector() {
    Eigen::VectorXd v(6);
    v << imu_acc, imu_gyro;
    return v;
  }

  // helper function, return a interpolated measurement between two measurements
  // for brevity we assume t1 < t < t2, caller should make sure this is true
  static std::shared_ptr<FootIMUMeasurement>
  interpolate(std::shared_ptr<FootIMUMeasurement> m1,
              std::shared_ptr<FootIMUMeasurement> m2, double t) {
    double t1 = m1->t;
    double t2 = m2->t;
    double alpha = (t - t1) / (t2 - t1);
    Eigen::Vector3d imu_acc = m1->imu_acc * (1 - alpha) + m2->imu_acc * alpha;
    Eigen::Vector3d imu_gyro =
        m1->imu_gyro * (1 - alpha) + m2->imu_gyro * alpha;
    return std::make_shared<FootIMUMeasurement>(t, imu_acc, imu_gyro, m1->id);
  }

  int id;
  Eigen::Vector3d imu_acc;
  Eigen::Vector3d imu_gyro;

  MeasureType type = FOOT_IMU;
  double t;
};
// measurement from the foot forces
// notice we assume they  are individual
class FootForceMeasurement : public Measurement {
public:
  FootForceMeasurement() {}

  FootForceMeasurement(double _t, Eigen::Vector3d _foot_force_xyz, int _id) {
    t = _t;
    foot_force_xyz = _foot_force_xyz;
    id = _id;
  }
  MeasureType getType() { return type; }
  double getTime() { return t; }
  Eigen::VectorXd getVector() { return foot_force_xyz; }

  // helper function, return a interpolated measurement between two measurements
  // for brevity we assume t1 < t < t2, caller should make sure this is true
  static std::shared_ptr<FootForceMeasurement>
  interpolate(std::shared_ptr<FootForceMeasurement> m1,
              std::shared_ptr<FootForceMeasurement> m2, double t) {
    double t1 = m1->t;
    double t2 = m2->t;
    double alpha = (t - t1) / (t2 - t1);
    Eigen::Vector3d foot_force_xyz =
        m1->foot_force_xyz * (1 - alpha) + m2->foot_force_xyz * alpha;
    return std::make_shared<FootForceMeasurement>(t, foot_force_xyz, m1->id);
  }

  int id;
  Eigen::Vector3d foot_force_xyz;
  MeasureType type = FOOT_FORCE;
  double t;
};
// direct pose measurement
// notice we assume they
class PoseMeasurement : public Measurement {
public:
  PoseMeasurement() {}

  PoseMeasurement(double _t, Eigen::Vector3d _pos, Eigen::Quaterniond _quat) {
    t = _t;
    pos = _pos;
    quat = _quat;
  }
  MeasureType getType() { return type; }
  double getTime() { return t; }
  Eigen::VectorXd getVector() {
    Eigen::VectorXd v(7);
    v << pos, quat.w(), quat.x(), quat.y(), quat.z();
    return v;
  }

  // helper function, return a interpolated measurement between two measurements
  // for brevity we assume t1 < t < t2, caller should make sure this is true
  // note that we need to interpolate the quaternion, we do so by get the lie
  // algebra of the differnce of the two quaternions and then interpolate the
  // lie algebra and then convert it back to quaternion
  static std::shared_ptr<PoseMeasurement>
  interpolate(std::shared_ptr<PoseMeasurement> m1,
              std::shared_ptr<PoseMeasurement> m2, double t) {
    double t1 = m1->t;
    double t2 = m2->t;
    double alpha = (t - t1) / (t2 - t1);
    Eigen::Vector3d pos = m1->pos * (1 - alpha) + m2->pos * alpha;
    Eigen::Quaterniond quat = m1->quat.slerp(alpha, m2->quat);
    return std::make_shared<PoseMeasurement>(t, pos, quat);
  }

  Eigen::Vector3d pos;
  Eigen::Quaterniond quat;
  MeasureType type = DIRECT_POSE;
  double t;
};

// https://stackoverflow.com/questions/16111337/declaring-a-priority-queue-in-c-with-a-custom-comparator
// The expression comp(a,b), where comp is an object of this type and a and b
// are elements in the container, shall return true if a is considered to go
// before b in the strict weak ordering the function defines.
class MeasurementCompare {
public:
  bool operator()(std::shared_ptr<Measurement> a,
                  std::shared_ptr<Measurement> b) {

    if (a == nullptr || b == nullptr) {
      return false;
    }
    if (a->getTime() > b->getTime()) {
      return true;
    } else {
      return false;
    }
  }
};

} // namespace SWE