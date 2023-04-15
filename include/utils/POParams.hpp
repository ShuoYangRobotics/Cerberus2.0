#pragma once
struct POParams {
  double init_cov = 0.05;
  double init_bias_cov = 1e-2;
  double init_body_height = 0.3;

  double proc_n_pos = 0.0005;
  double proc_n_vel_xy = 0.005;
  double proc_n_vel_z = 0.005;
  double proc_n_ang = 1e-7;
  double proc_n_foot_pos = 1e-4;
  double proc_n_foot_vel = 5;
  double proc_n_ba = 1e-4;
  double proc_n_bg = 1e-5;
  double proc_n_foot1_ba = 1e-4;
  double proc_n_foot2_ba = 1e-4;
  double proc_n_foot3_ba = 1e-4;
  double proc_n_foot4_ba = 1e-4;

  double ctrl_n_acc = 1e-1;
  double ctrl_n_gyro = 1e-3;
  double ctrl_n_foot1_acc = 1;
  double ctrl_n_foot2_acc = 1;
  double ctrl_n_foot3_acc = 1;
  double ctrl_n_foot4_acc = 1;

  double meas_n_fk_pos = 0.001;
  double meas_n_fk_vel = 0.05;
  double meas_n_foot_height = 0.001;
  double meas_n_rolling_vel = 0.01;
};