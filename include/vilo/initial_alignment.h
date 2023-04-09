/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <ros/ros.h>

#include "factor/imu_factor.hpp"
#include "featureTracker/feature_manager.h"
#include "utils/tic_toc.h"
#include "utils/vins_utility.h"

using namespace Eigen;
using namespace std;

class ImageFrame {
public:
  ImageFrame(){};
  ImageFrame(
      const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &_points,
      double _t)
      : t{_t}, is_key_frame{false} {
    points = _points;
  };
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> points;
  double t;
  Matrix3d R;
  Vector3d T;
  std::shared_ptr<IntegrationBase> pre_integration;
  bool is_key_frame;
};
void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame,
                        Vector3d *Bgs);
bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d *Bgs,
                        Vector3d &g, VectorXd &x);