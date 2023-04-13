#pragma once

#include "Measurement.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <queue>
#include <stdexcept>

namespace SWE {
/*
 * The MeasureQueue has a "main sensor", default to be the body IMU.
 * when we call push (there are several types of push), any measurement will be
 * pushed into it; pop and getHorizon works measurement by measurement. when we
 * use getMainHorizon(), popMain(), they counter numbers of main sensor
 * measurements
 */
typedef std::priority_queue<std::shared_ptr<Measurement>,
                            std::vector<std::shared_ptr<Measurement>>,
                            MeasurementCompare>
    pq_meas_type;
class MeasureQueue {
public:
  MeasureQueue() {}
  MeasureQueue(MeasureType type) {
    main_type = type;
    current_main_size = 0;
  }
  // destructor, clean data structure
  ~MeasureQueue() {
    // vec_meas only keeps pointer, do not need to clean it

    // all elements in pq_meas must be properly deleted
    // but the underlying Measurement data are still kept because other
    // MeasureQueue may refer to them
    while (!pq_meas.empty()) {
      pq_meas.pop();
    }
  }

  double size() { return pq_meas.size(); }

  std::shared_ptr<Measurement> top() { return pq_meas.top(); }

  bool empty() { return pq_meas.empty(); }

  double latestTime() { return newestElementTime; }
  double oldestTime() { return pq_meas.top()->getTime(); }

  // get the horizon time of the queue, which equals to the time of the
  // first element minus the time of the last element
  double timeSpan() {
    if (pq_meas.empty()) {
      return 0.0;
    } else {
      return newestElementTime - pq_meas.top()->getTime();
    }
  }

  double mainSize() { return current_main_size; }

  // remove a given number of elements by pop
  void pop(int num_elems = 1) {
    int counter = 0;
    while (!pq_meas.empty() && counter < num_elems) {
      std::shared_ptr<Measurement> tmp = pq_meas.top();
      if (tmp->getType() == main_type) {
        current_main_size--;
      }
      tmp.reset();
      pq_meas.pop();
      counter++;
    }
    return;
  }

  /*
   * a list of push operations, they all align with measurement constructor
   * functions
   */
  // BodyIMUMeasurement
  bool push(double _t, Eigen::Vector3d _imu_acc, Eigen::Vector3d _imu_gyro) {
    std::shared_ptr<Measurement> tmp =
        std::make_shared<BodyIMUMeasurement>(_t, _imu_acc, _imu_gyro);
    newestElementTime = _t;
    pq_meas.push(tmp);

    if (tmp->getType() == main_type) {
      current_main_size++;
    }
    return true;
  }

  // LegMeasurement
  bool push(double _t, Eigen::Matrix<double, 12, 1> _joint_pos,
            Eigen::Matrix<double, 12, 1> _joint_vel) {
    std::shared_ptr<Measurement> tmp =
        std::make_shared<LegMeasurement>(_t, _joint_pos, _joint_vel);
    newestElementTime = _t;
    pq_meas.push(tmp);
    if (tmp->getType() == main_type) {
      current_main_size++;
    }
    return true;
  }

  // BodyIMUMeasurement & LegMeasurement
  bool push(double _t, Eigen::Vector3d _imu_acc, Eigen::Vector3d _imu_gyro,
            Eigen::Matrix<double, 12, 1> _joint_pos,
            Eigen::Matrix<double, 12, 1> _joint_vel) {
    std::shared_ptr<Measurement> tmp1 =
        std::make_shared<BodyIMUMeasurement>(_t, _imu_acc, _imu_gyro);
    std::shared_ptr<Measurement> tmp2 =
        std::make_shared<LegMeasurement>(_t, _joint_pos, _joint_vel);
    newestElementTime = _t;
    pq_meas.push(tmp1);
    if (tmp1->getType() == main_type) {
      current_main_size++;
    }
    pq_meas.push(tmp2);
    if (tmp2->getType() == main_type) {
      current_main_size++;
    }
    return true;
  }

  // LegMeasurement with tau
  bool push(double _t, Eigen::Matrix<double, 12, 1> _joint_pos,
            Eigen::Matrix<double, 12, 1> _joint_vel,
            Eigen::Matrix<double, 12, 1> _joint_tau) {
    std::shared_ptr<Measurement> tmp = std::make_shared<LegMeasurement>(
        _t, _joint_pos, _joint_vel, _joint_tau);
    newestElementTime = _t;
    pq_meas.push(tmp);
    if (tmp->getType() == main_type) {
      current_main_size++;
    }
    return true;
  }

  // BodyIMUMeasurement & LegMeasurement with tau
  bool push(double _t, Eigen::Vector3d _imu_acc, Eigen::Vector3d _imu_gyro,
            Eigen::Matrix<double, 12, 1> _joint_pos,
            Eigen::Matrix<double, 12, 1> _joint_vel,
            Eigen::Matrix<double, 12, 1> _joint_tau) {
    std::shared_ptr<Measurement> tmp1 =
        std::make_shared<BodyIMUMeasurement>(_t, _imu_acc, _imu_gyro);
    std::shared_ptr<Measurement> tmp2 = std::make_shared<LegMeasurement>(
        _t, _joint_pos, _joint_vel, _joint_tau);
    newestElementTime = _t;
    pq_meas.push(tmp1);
    if (tmp1->getType() == main_type) {
      current_main_size++;
    }
    pq_meas.push(tmp2);
    if (tmp2->getType() == main_type) {
      current_main_size++;
    }
    return true;
  }

  // FootIMUMeasurement
  bool push(double _t, Eigen::Vector3d _imu_acc, Eigen::Vector3d _imu_gyro,
            int _id) {
    std::shared_ptr<Measurement> tmp =
        std::make_shared<FootIMUMeasurement>(_t, _imu_acc, _imu_gyro, _id);
    newestElementTime = _t;
    pq_meas.push(tmp);
    if (tmp->getType() == main_type) {
      current_main_size++;
    }
    return true;
  }

  // FootForceMeasurement
  bool push(double _t, Eigen::Vector3d _foot_force_xyz, int _id) {
    std::shared_ptr<Measurement> tmp =
        std::make_shared<FootForceMeasurement>(_t, _foot_force_xyz, _id);
    newestElementTime = _t;
    pq_meas.push(tmp);
    if (tmp->getType() == main_type) {
      current_main_size++;
    }
    return true;
  }

  // FootForceALLMeasurement
  bool push(double _t, Eigen::Vector4d _foot_force_z) {
    std::shared_ptr<Measurement> tmp =
        std::make_shared<FootForceALLMeasurement>(_t, _foot_force_z);
    newestElementTime = _t;
    pq_meas.push(tmp);
    if (tmp->getType() == main_type) {
      current_main_size++;
    }
    return true;
  }

  // PoseMeasurement
  bool push(double _t, Eigen::Vector3d _pos, Eigen::Quaterniond _quat) {
    std::shared_ptr<Measurement> tmp =
        std::make_shared<PoseMeasurement>(_t, _pos, _quat);
    newestElementTime = _t;
    pq_meas.push(tmp);
    if (tmp->getType() == main_type) {
      current_main_size++;
    }
    return true;
  }

  // get measurement within the horizon
  // e.g.   if queue contains
  //        IMU-IMU-DIRECT_POSE-IMU-IMU-DIRECT_POSE-IMU
  // then getHorizon(5) should return
  //                DIRECT_POSE-IMU-IMU-DIRECT_POSE-IMU
  std::vector<std::shared_ptr<Measurement>> getHorizon(int horizon_length) {
    if (horizon_length > pq_meas.size()) {
      // something is wrong, throw error
      std::cout << "horizon cannot be longer than the pq_meas.size()"
                << std::endl;
      throw 203;
    }
    if (horizon_length < 2) {
      // something is wrong, throw error
      std::cout << "horizon cannot be shorter than 2" << std::endl;
      throw 204;
    }
    std::vector<std::shared_ptr<Measurement>> tmp_vec_meas;
    tmp_vec_meas.clear();

    pq_meas_type tmp_pq_meas(pq_meas); // copy a pq_meas_type
    // pop until only horizon_length elements left
    while (tmp_pq_meas.size() > horizon_length) {
      tmp_pq_meas.pop();
    }
    // then save the rest horizon_length elements to tmp_vec_meas
    while (!tmp_pq_meas.empty()) {
      std::shared_ptr<Measurement> tmp = tmp_pq_meas.top();
      tmp_vec_meas.push_back(tmp);
      tmp_pq_meas.pop();
    }
    return tmp_vec_meas;
  }

  // remove main measurements and all non-main measurements to their left
  void popMain(int num_elems = 1) {
    if (current_main_size < num_elems) {
      std::cout << "cannot pop this many main measurements" << std::endl;
      throw 206;
    }
    int counter = 0;
    while (!pq_meas.empty() && counter < num_elems) {
      while (pq_meas.top()->getType() != main_type) {
        std::shared_ptr<Measurement> tmp = pq_meas.top();
        tmp.reset();
        pq_meas.pop();
      }
      // now this is a main measurement
      std::shared_ptr<Measurement> tmp = pq_meas.top();
      tmp.reset();
      pq_meas.pop();
      current_main_size--;
      while (pq_meas.top()->getType() != main_type) {
        std::shared_ptr<Measurement> tmp = pq_meas.top();
        tmp.reset();
        pq_meas.pop();
      }
      counter++;
    }
    return;
  }

  // remove all measurements that is older than the input time
  void popUntil(double time) {
    while (!pq_meas.empty() && pq_meas.top()->getTime() < time) {
      std::shared_ptr<Measurement> tmp = pq_meas.top();
      if (tmp->getType() == main_type) {
        current_main_size--;
      }
      tmp.reset();
      pq_meas.pop();
    }
    return;
  }

  // get measurement within the horizon defined by main sensor types
  // e.g.   if queue contains
  //        IMU-IMU-DIRECT_POSE-IMU-IMU-DIRECT_POSE-IMU
  // then getHorizonMain(5) should return
  //        IMU-IMU-DIRECT_POSE-IMU-IMU-DIRECT_POSE-IMU
  std::vector<std::shared_ptr<Measurement>> getHorizonMain(int horizon_length) {
    if (horizon_length > pq_meas.size()) {
      // something is wrong, throw error
      std::cout << "horizon cannot be longer than the pq_meas.size()"
                << std::endl;
      throw 203;
    }
    if (horizon_length < 2) {
      // something is wrong, throw error
      std::cout << "horizon cannot be shorter than 2" << std::endl;
      throw 204;
    }
    if (horizon_length > current_main_size) {
      // something is wrong, throw error
      std::cout << "horizon cannot be longer than the current_main_size"
                << std::endl;
      throw 205;
    }
    std::vector<std::shared_ptr<Measurement>> tmp_vec_meas;
    tmp_vec_meas.clear();
    pq_meas_type tmp_pq_meas = pq_meas; // copy a pq_meas_type

    // use a counter to keep track how many main element left
    int left_main_number = current_main_size;
    while (left_main_number >= horizon_length) {
      // pop until top element has main type
      while (tmp_pq_meas.top()->getType() != main_type) {
        tmp_pq_meas.pop();
      }
      if (left_main_number == horizon_length) {
        break;
      } else {
        tmp_pq_meas.pop();
        left_main_number--;
      }
    }
    // then save all of the rest elements to tmp_vec_meas
    while (!tmp_pq_meas.empty()) {
      std::shared_ptr<Measurement> tmp = tmp_pq_meas.top();
      tmp_vec_meas.push_back(tmp);
      tmp_pq_meas.pop();
    }
    return tmp_vec_meas;
  }

  // this function get the lastest main measurement and all other measurements
  // since the second last main measurement
  std::vector<std::shared_ptr<Measurement>> getLastestMain() {
    std::vector<std::shared_ptr<Measurement>> horiz_meas = getHorizonMain(2);
    horiz_meas.erase(horiz_meas.begin());
    return horiz_meas;
  }

  // dump elements to vec_meas with
  void dump_vec() {
    vec_meas.clear();
    pq_meas_type tmp_pq_meas(pq_meas); // copy a pq_meas_type
    while (!tmp_pq_meas.empty()) {
      std::shared_ptr<Measurement> tmp = tmp_pq_meas.top();
      vec_meas.push_back(tmp);
      tmp_pq_meas.pop();
    }
  }

  int print_queue() {
    dump_vec();
    for (std::shared_ptr<Measurement> element : vec_meas) {
      std::cout << "(" << std::setprecision(15) << element->getTime() << "\t,"
                << element->getType() << ") ";
    }
    std::cout << std::endl;
    return vec_meas.size();
  }

  // helper function
  int print_queue(std::vector<std::shared_ptr<Measurement>> vec) {
    for (std::shared_ptr<Measurement> element : vec) {
      if (element == nullptr) {
        std::cout << "nullptr" << std::endl;
        throw 208;
      }
      std::cout << "(" << std::setprecision(15) << element->getTime() << "\t,"
                << element->getType() << ") ";
    }
    std::cout << std::endl;
    return vec.size();
  }

  // helper function intepolate between two measurements
  std::shared_ptr<Measurement>
  interpolate_two(double t, std::shared_ptr<Measurement> m1,
                  std::shared_ptr<Measurement> m2) {
    // first make sure the time is within the range
    if (t < m1->getTime() || t > m2->getTime()) {
      std::cout << "time is out of range" << std::endl;
      throw 207;
    }
    // if t is the same as one of the element, return that element
    if (t == m1->getTime()) {
      // copy the measurement and return a shared pointer
      return m1;
    } else if (t == m2->getTime()) {
      return m2;
    }
    // otherwise interpolate, here we need to cast m1 and m2 to the correct
    // measurement type and call the interpolate function
    if (m1->getType() == BODY_IMU) {
      return BodyIMUMeasurement::interpolate(
          std::dynamic_pointer_cast<BodyIMUMeasurement>(m1),
          std::dynamic_pointer_cast<BodyIMUMeasurement>(m2), t);

    } else if (m1->getType() == LEG) {
      return LegMeasurement::interpolate(
          std::dynamic_pointer_cast<LegMeasurement>(m1),
          std::dynamic_pointer_cast<LegMeasurement>(m2), t);

    } else if (m1->getType() == FOOT_IMU) {
      return FootIMUMeasurement::interpolate(
          std::dynamic_pointer_cast<FootIMUMeasurement>(m1),
          std::dynamic_pointer_cast<FootIMUMeasurement>(m2), t);

    } else if (m1->getType() == FOOT_FORCE) {
      return FootForceMeasurement::interpolate(
          std::dynamic_pointer_cast<FootForceMeasurement>(m1),
          std::dynamic_pointer_cast<FootForceMeasurement>(m2), t);

    } else if (m1->getType() == FOOT_FORCE_ALL) {
      return FootForceALLMeasurement::interpolate(
          std::dynamic_pointer_cast<FootForceALLMeasurement>(m1),
          std::dynamic_pointer_cast<FootForceALLMeasurement>(m2), t);

    } else if (m1->getType() == DIRECT_POSE) {
      return PoseMeasurement::interpolate(
          std::dynamic_pointer_cast<PoseMeasurement>(m1),
          std::dynamic_pointer_cast<PoseMeasurement>(m2), t);

    } else {
      std::cout << "measurement type not supported" << std::endl;
      throw 206;
    }
  }

  // helper function, intepolate among a vector of measurements
  std::shared_ptr<Measurement>
  interpolate_list(double t, std::vector<std::shared_ptr<Measurement>> &vec) {
    // first make sure the time is within the range
    if (t < vec.front()->getTime() || t > vec.back()->getTime()) {
      std::cout << "time is out of range" << std::endl;
      throw 207;
    }
    if (vec.size() == 1) {
      return vec.front();
    }
    // find the two element that is closest to t, if t is the same as one of the
    // element, return that element
    int i = 0;
    while (vec[i]->getTime() < t) {
      i++;
    }
    if (vec[i]->getTime() == t) {
      return vec[i];
    }
    // else call the interpolate_two function
    return interpolate_two(t, vec[i - 1], vec[i]);
  }

  // helper function interpolate with just a given time
  // caller must ensure that the time is within the range
  std::shared_ptr<Measurement> interpolate(double t) {
    dump_vec();
    return interpolate_list(t, vec_meas);
  }

private:
  pq_meas_type pq_meas;
  // only help print queue and construct SWE
  std::vector<std::shared_ptr<Measurement>> vec_meas;

  // keep track of current number and maximum number of elements
  // maybe let out users handle this?
  int max_queue_size;

  // keey track of main sensor type and number
  int current_main_size;

  // this is the time of the newest element in the queue
  double newestElementTime;

  MeasureType main_type = BODY_IMU;
};

} // namespace SWE
