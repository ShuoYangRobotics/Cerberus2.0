// standard C
#include <signal.h>
#include <thread>

// ros related
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

// project files
#include "fusion/VILOFusion.hpp"
#include "utils/visualization.h"

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
  std::cout << "Caught signal " << signum << std::endl;
  // Terminate program
  exit(signum);
}

int main(int argc, char **argv) {
  // Register signal and signal handler
  signal(SIGINT, signal_callback_handler);
  ros::init(argc, argv, "cerberus2");
  ros::NodeHandle nh;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);

  // load parameters
  Utils::readParametersROS(nh);

  if (argc != 2) {
    printf("please input: rosrun cerberus2 cerberus2_main [config "
           "file] \n"
           "for example: rosrun cerberus2 cerberus2_main "
           "/home/EstimationUser/estimation_ws/src/cerberus2/config/go1_config/"
           "hardware_go1_vilo_config.yaml \n");
    return 1;
  }
  std::string config_file = argv[1];
  Utils::readParametersFile(config_file);

  // register visualization publishers
  Utils::registerPub(nh);

  /* create an estimator object */
  VILOFusion vilo_fusion(nh);

  ros::spin();
  return 0;
}