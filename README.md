# Cerberus 2.0
A precise low-drift Visual-Inertial-Leg Odometry for legged robots

This is an improved version of [Cerberus](https://github.com/ShuoYangRobotics/Cerberus). It is based on the following publications:

* **Multi-IMU Proprioceptive Odometry for Legged Robots**, Yang, Shuo and Zhang, Zixin and Bokser, Benjamin,  and Manchester, Zachary. (IROS 2023, Best Paper Finalist), [pdf](https://roboticexplorationlab.org/papers/foot_imu_iros2023.pdf)

* **Cerberus: Low-Drift Visual-Inertial-Leg Odometry For Agile Locomotion**, Yang, Shuo and Zhang, Zixin and Fu, Zhengyu and Manchester, Zachary. (ICRA 2023), [pdf](https://ieeexplore.ieee.org/document/10160486)

* **Online Kinematic Calibration for Legged Robots**, Yang, Shuo and Choset, Howie and Manchester, Zachary, IEEE Robotics and Automation Letters (& IROS 2022), [pdf](https://ieeexplore.ieee.org/abstract/document/9807408)


If you use Cerberus or Cerberus2.0 for your academic research, please cite at least one of our related papers.


# Installation
The code is tested on Windows 11 with WSL2 and Ubuntu 20.04.2 LTS. It should also work on Ubuntu 18.04.5 LTS because the code is based on docker. Your computer should install docker (Linux) or Docker Desktop+WSL2 (windows). 

Open the repo folder in VSCode, use VSCode's "remote container" feature to open the repo in a docker container. The docker container will automatically install all the dependencies. The configuration file is in the folder ".devcontainer". Depends on your host computer type, copy either ".devcontainer/devcontainer-ubuntu.json" or ".devcontainer/devcontainer-wsl.json" to ".devcontainer/devcontainer.json", which is the configuration file for VSCode's "remote container" feature.

The docker config file automatically mount the project folder to ~/estimate_ws/src/cerberus2 inside the docker container. After the docker container is built, run "catkin build" to compile cerberus2.0.
