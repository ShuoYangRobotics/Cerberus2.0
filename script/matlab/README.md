## Dataset explanation
Google drive folder for dataset: [here](https://drive.google.com/drive/folders/1n7mgTQ3QK3ZFJXIftBbQDHURixudRSQS)

Each subfolder DATASET_FOLDER_NAME should have a DATASET_NAME.bag file and a DATASET_NAME.mat file. 

The DATASET_NAME.bag contains
```
topics:      /WT901_47_Data                           8841 msgs    : sensor_msgs/Imu       
             /WT901_48_Data                           8840 msgs    : sensor_msgs/Imu       
             /WT901_49_Data                           8840 msgs    : sensor_msgs/Imu       
             /WT901_50_Data                           8841 msgs    : sensor_msgs/Imu       
             /camera_forward/infra1/image_rect_raw     694 msgs    : sensor_msgs/Image     
             /camera_forward/infra2/image_rect_raw     694 msgs    : sensor_msgs/Image     
             /unitree_hardware/imu                   18081 msgs    : sensor_msgs/Imu       
             /unitree_hardware/joint_foot            18080 msgs    : sensor_msgs/JointState
```

If the bag is an indoor dataset, then it should also contain
```
             /natnet_ros/Shuo_Go1/pose                4599 msgs    : geometry_msgs/PoseStamped
```

The DATASET_NAME.mat contains MATLAB mobile recorded iphone GPS and IMU data. It can be used to generate the ground truth trajectory in outdoor datasets. 

## Dataset setup instructions
Assume you have installed ROS and created a catkin workspace to install Cerberus and Cerberus2.0 in ~/vilo_ws/src. We create a folder ~/vilo_ws/bags to store the dataset. 
```
~/vilo_ws/bags
├── DATASET_FOLDER_NAME 1
│   ├── DATASET_NAME.bag
│   ├── DATASET_NAME.mat
├── DATASET_FOLDER_NAME 2
│
...
```
Then we use various shell scripts in Cerberus2.0/launch to run Cerberus and Cerberus 2.0 on the dataset.

We must run vilo_outdoor_compare.sh with arguments
```
./vilo_outdoor_dataset_multi_runs.sh DATASET_NAME 46 DATASET_FOLDER_NAME
```
Notice **46** is the dataset time which can be seen from rosbag info DATASET_NAME.bag.

For example, using May 25 datasets, we run
```
./vilo_outdoor_dataset_multi_runs.sh 20230525_frick_park_arch_short 46 230525
./vilo_indoor_dataset_multi_runs.sh 20230525_new_IMU_round 26 230525
```

After the shell script finishes execution, there will be output files in the following folder
```
~/vilo_ws/bags
├── DATASET_FOLDER_NAME 1
│   ├── DATASET_NAME.bag
│   ├── DATASET_NAME.mat
├── DATASET_FOLDER_NAME 2
├── cerberus2_output
│   ├── DATASET_NAME
│   │   ├── gt-DATASET_NAME.csv
│   │   ├── mipo-DATASET_NAME.csv
│   │   ├── sipo-DATASET_NAME.csv
│   │   ├── vilo-m-DATASET_NAME.csv
│   │   ├── vilo-s-DATASET_NAME.csv
│   │   ├── vilo-tm-n-DATASET_NAME.csv
│   │   ├── vilo-tm-y-DATASET_NAME.csv
│   │   └── vio-DATASET_NAME.csv
├── cerberus_output
│   ├── DATASET_NAME
│   │   ├── cerberus-wb-DATASET_NAME.csv
│   │   ├── cerberus-wob-DATASET_NAME.csv

```

## Data process instructions
We visualize and analyze dataset using scripts in **Cerberus2.0/script/matlab**
```
outdoor_cerberus12_single_dataset_compare.m
```
Change section 2 to the dataset you want to visualize, modify other values if necessary, and then run the script.
A figure compares the ground truth, cerberus variants, and cerberus2.0 variants will be generated. 
