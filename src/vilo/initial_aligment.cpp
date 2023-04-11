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

#include "vilo/initial_alignment.h"

void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame,
                        Vector3d *Bgs) {
  Matrix3d A;
  Vector3d b;
  Vector3d delta_bg;
  A.setZero();
  b.setZero();
  map<double, ImageFrame>::iterator frame_i;
  map<double, ImageFrame>::iterator frame_j;
  for (frame_i = all_image_frame.begin();
       next(frame_i) != all_image_frame.end(); frame_i++) {
    frame_j = next(frame_i);
    MatrixXd tmp_A(3, 3);
    tmp_A.setZero();
    VectorXd tmp_b(3);
    tmp_b.setZero();
    Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
    tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(
        O_R, O_BG);
    tmp_b =
        2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
    A += tmp_A.transpose() * tmp_A;
    b += tmp_A.transpose() * tmp_b;
  }
  delta_bg = A.ldlt().solve(b);
  ROS_WARN_STREAM("gyroscope bias initial calibration "
                  << delta_bg.transpose());

  for (int i = 0; i <= WINDOW_SIZE; i++)
    Bgs[i] += delta_bg;

  for (frame_i = all_image_frame.begin();
       next(frame_i) != all_image_frame.end(); frame_i++) {
    frame_j = next(frame_i);
    frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
  }
}
