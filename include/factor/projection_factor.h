/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ros/assert.h>

#include "utils/parameters.hpp"
#include "utils/tic_toc.h"
#include "utils/vins_utility.h"

class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1> {
public:
  ProjectionFactor(const Eigen::Vector3d &_pts_i,
                   const Eigen::Vector3d &_pts_j);
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const;
  void check(double **parameters);

  Eigen::Vector3d pts_i, pts_j;
  Eigen::Matrix<double, 2, 3> tangent_base;
  static Eigen::Matrix2d sqrt_info;
  static double sum_t;
};
