#receive a string as dataset name from argument

dataset_name=$1
dataset_duration=$2
dataset_foldername=$3

ws_root='/home/rosie2/vilo_dev/vilo_ws'

#ls the dataset
bag_path="$ws_root/bags/$dataset_foldername/$dataset_name.bag"
echo $bag_path

# first, run cerberus2 
temp_config="/tmp/temp_run.yaml"

cp $ws_root/src/Cerberus2.0/config/go1_config/go1_realsense_left.yaml /tmp/go1_realsense_left.yaml
cp $ws_root/src/Cerberus2.0/config/go1_config/go1_realsense_right.yaml /tmp/go1_realsense_right.yaml
cp $ws_root/src/Cerberus2.0/config/go1_config/hardware_go1_vilo_config_outdoor.yaml "$temp_config"

sed -i "s/\(dataset_name: \)\"lab\"/\1\"$dataset_name\"/" "$temp_config"

# set output folder, create $ws_root/bags/cerberus2_output/$dataset_foldername if not exist
output_folder="$ws_root/bags/cerberus2_output/$dataset_name"
mkdir -p $output_folder

# use sed change first appeared string 'OUTPUT_FOLDER' to  $dataset_foldername in $temp_config
sed -i "0,/OUTPUT_FOLDER/s//${dataset_name}/" "$temp_config"


# launch the vilo node with kf_type and vilo_fusion_type:           0 0
roslaunch cerberus2 vilo_dataset_multi_runs.launch dataset_name:="$dataset_name" dataset_duration:="$dataset_duration" dataset_foldername:="$dataset_foldername"

# launch the vilo node with different kf_type and vilo_fusion_type: 1 0
sed -i 's/\(kf_type: \)0/\11/' "$temp_config"
roslaunch cerberus2 vilo_dataset_multi_runs.launch dataset_name:="$dataset_name" dataset_duration:="$dataset_duration" dataset_foldername:="$dataset_foldername"

# launch the vilo node with different kf_type and vilo_fusion_type: 1 1
sed -i 's/\(vilo_fusion_type: \)0/\11/' "$temp_config"
roslaunch cerberus2 vilo_dataset_multi_runs.launch dataset_name:="$dataset_name" dataset_duration:="$dataset_duration" dataset_foldername:="$dataset_foldername"

# launch the vilo node with different kf_type and vilo_fusion_type: 0 1
sed -i 's/\(kf_type: \)1/\10/' "$temp_config"
roslaunch cerberus2 vilo_dataset_multi_runs.launch dataset_name:="$dataset_name" dataset_duration:="$dataset_duration" dataset_foldername:="$dataset_foldername"

# launch the vilo node with different kf_type and vilo_fusion_type: 0 2  

sed -i 's/\(vilo_fusion_type: \)1/\12/' "$temp_config"
roslaunch cerberus2 vilo_dataset_multi_runs.launch dataset_name:="$dataset_name" dataset_duration:="$dataset_duration" dataset_foldername:="$dataset_foldername"

# launch the vilo node with different kf_type and vilo_fusion_type: 0 2, no exstimate kinematic  # should be the best ones
sed -i 's/\(estimate_kinematic: \)1/\10/' "$temp_config"
roslaunch cerberus2 vilo_dataset_multi_runs.launch dataset_name:="$dataset_name" dataset_duration:="$dataset_duration" dataset_foldername:="$dataset_foldername"

# finally remove the temp file
rm "$temp_config"


# run cerberus comparison
temp_config="/tmp/temp_run_cerberus.yaml"
cp $ws_root/src/Cerberus/config/go1_config/go1_realsense_left.yaml /tmp/go1_realsense_left.yaml
cp $ws_root/src/Cerberus/config/go1_config/go1_realsense_right.yaml /tmp/go1_realsense_right.yaml
cp $ws_root/src/Cerberus/config/go1_config/hardware_go1_vilo_config_outdoor.yaml "$temp_config"

# set output folder, create $ws_root/bags/cerberus2_output/$dataset_foldername if not exist
output_folder="$ws_root/bags/cerberus_output/$dataset_name"
mkdir -p $output_folder

# use sed change first appeared string 'OUTPUT_FOLDER' to  $dataset_foldername in $temp_config
sed -i "0,/OUTPUT_FOLDER/s//${dataset_name}/" "$temp_config"

sed -i "s/\(dataset_name: \)\"lab\"/\1\"$dataset_name\"/" "$temp_config"
roslaunch vilo cerberus_auto.launch dataset_name:="$dataset_name" dataset_duration:="$dataset_duration" dataset_foldername:="$dataset_foldername" 

sed -i 's/\(optimize_leg_bias: \)0/\11/' "$temp_config"
roslaunch vilo cerberus_auto.launch dataset_name:="$dataset_name" dataset_duration:="$dataset_duration" dataset_foldername:="$dataset_foldername"

# finally remove the temp file
rm "$temp_config"