#receive a string as dataset name from argument

dataset_name=$1
dataset_duration=$2

#ls the dataset
bag_path="/home/rosie2/vilo_dev/vilo_ws/bags/230517_sqh_mocap/$dataset_name.bag"
echo $bag_path


temp_config="/tmp/temp_run.yaml"

cp /home/rosie2/vilo_dev/vilo_ws/src/Cerberus2.0/config/go1_config/go1_realsense_left.yaml /tmp/go1_realsense_left.yaml
cp /home/rosie2/vilo_dev/vilo_ws/src/Cerberus2.0/config/go1_config/go1_realsense_right.yaml /tmp/go1_realsense_right.yaml
cp /home/rosie2/vilo_dev/vilo_ws/src/Cerberus2.0/config/go1_config/hardware_go1_vilo_config_base.yaml "$temp_config"

sed -i "s/\(dataset_name: \)\"lab\"/\1\"$dataset_name\"/" "$temp_config"

# launch the vilo node with kf_type and vilo_fusion_type:           0 0
roslaunch cerberus2 vilo_auto.launch dataset_name:="$dataset_name" dataset_duration:="$dataset_duration"

# launch the vilo node with different kf_type and vilo_fusion_type: 1 0
sed -i 's/\(kf_type: \)0/\11/' "$temp_config"
roslaunch cerberus2 vilo_auto.launch dataset_name:="$dataset_name" dataset_duration:="$dataset_duration"

# launch the vilo node with different kf_type and vilo_fusion_type: 1 1
sed -i 's/\(vilo_fusion_type: \)0/\11/' "$temp_config"
roslaunch cerberus2 vilo_auto.launch dataset_name:="$dataset_name" dataset_duration:="$dataset_duration"

# launch the vilo node with different kf_type and vilo_fusion_type: 0 1
sed -i 's/\(kf_type: \)1/\10/' "$temp_config"
roslaunch cerberus2 vilo_auto.launch dataset_name:="$dataset_name" dataset_duration:="$dataset_duration"

# launch the vilo node with different kf_type and vilo_fusion_type: 0 2  
sed -i 's/\(vilo_fusion_type: \)1/\12/' "$temp_config"
roslaunch cerberus2 vilo_auto.launch dataset_name:="$dataset_name" dataset_duration:="$dataset_duration"

# launch the vilo node with different kf_type and vilo_fusion_type: 0 2, no exstimate kinematic  # should be the best ones
sed -i 's/\(estimate_kinematic: \)1/\10/' "$temp_config"
roslaunch cerberus2 vilo_auto.launch dataset_name:="$dataset_name" dataset_duration:="$dataset_duration"

# finally remove the temp file
rm "$temp_config"
