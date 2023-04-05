// standard C
#include <signal.h>
#include <thread>

// ros related
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

// project files
#include "fusion/VILOFusion.hpp"

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
  std::cout << "Caught signal " << signum << std::endl;
  // Terminate program
  exit(signum);
}

int main(int argc, char** argv) {
  // Register signal and signal handler
  signal(SIGINT, signal_callback_handler);
  ros::init(argc, argv, "cerberus2");
  ros::NodeHandle nh;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  /* create an estimator object */
  VILOFusion vilo_fusion(nh);

  ros::spin();
  return 0;
}