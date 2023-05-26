% add mobile gps process code
addpath('mobile_gps_process/')

% prepare necessary file paths
BAG_FOLDER_PATH = '/home/shuoyang/Documents/vilo_dev/vilo_ws/bags/';
CERBERUS_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus_output/'];
CERBERUS2_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus2_output/'];

%% only need to change these three usually
DATASET_FOLDER_NAME = '230525';
DATASET_NAME = '20230525_frick_park_arch_short';
DATASET_TIME = 46;
GT_TIME_OFFSET = 20; % how much GT is longer than the robot dataset
GT_YAW_OFFSET = 10; % deg of gt

DATASET_X_RANGE = [-1 25];
DATASET_Y_RANGE = [-2 7];
DATASET_Z_RANGE = [-5 5];

%% if mobile exists
% process mobile data
mobile_gps_file_name = [BAG_FOLDER_PATH,'/',DATASET_FOLDER_NAME,'/',...
    DATASET_NAME,'.mat'];
has_mobile_gt = 0;
if isfile(mobile_gps_file_name)
    gps_position = position_filter(mobile_gps_file_name, DATASET_TIME+GT_TIME_OFFSET, GT_YAW_OFFSET);
    has_mobile_gt = 1;
else
    has_mobile_gt = 0;
end

%% prepare figure
figure(1);clf

% plot gps_position as gt
traj_types =      {     'gt'};
traj_colors =     {'#0072BD'};
plot3(gps_position(:,1),gps_position(:,2),gps_position(:,3), 'Color',traj_colors{1}); hold on;

%% plot cerberus2
CERBERUS2_OUTPUT_DATASET_FOLDER_PATH = [CERBERUS2_OUTPUT_FOLDER_PATH,DATASET_NAME,'/'];
% check whether CERBERUS2_OUTPUT_DATASET_FOLDER_PATH is emppty

% look at src/utils/parameters.cpp for possible types
% traj_types = {'gt','mipo','sipo','vio','vilo-m','vilo-s'};
cerberus2_traj_types = {'mipo','sipo','vio','vilo-m','vilo-s','vilo-tm-n','vilo-tm-y'};
cerberus2_traj_colors ={'#0072BD','#D95319','#EDB120','#7E2F8E','#77AC30','#4DBEEE','#000000'};

% traj_types = {'mipo','vilo-tm-n'};
cerberus2_total_types = size(cerberus2_traj_types,2);

plot_start = 0;
plot_end = DATASET_TIME;
% read data
traj_data = cell(1, cerberus2_total_types);
for i=1:cerberus2_total_types
    csv_file_full_name = [CERBERUS2_OUTPUT_DATASET_FOLDER_PATH,...
        cerberus2_traj_types{i},'-',DATASET_NAME,'.csv'];
    if isfile(csv_file_full_name)
        csv_file_full_name
        traj_data{i} = readmatrix(csv_file_full_name);
    else 
        csv_file_full_name
        disp({csv_file_full_name, ' is not valid'})
    end
end
 
traj_t0 = zeros(1, cerberus2_total_types);
traj_time = cell(1, cerberus2_total_types);
traj_pos = cell(1, cerberus2_total_types);
traj_euler = cell(1, cerberus2_total_types);

for i=1:cerberus2_total_types
    i
    [traj_t0(i), traj_time{i}, traj_pos{i}, traj_euler{i}, ~] =...
        parse_cerberus2_data(traj_data{i}, plot_start, plot_end);
end

traj_t0 = traj_t0 - min(traj_t0);
for i=1:cerberus2_total_types
traj_time{i} = traj_time{i}+traj_t0(i);
end



for i=1:cerberus2_total_types
 plot3(traj_pos{i}(:,1),traj_pos{i}(:,2),traj_pos{i}(:,3),'Color',cerberus2_traj_colors{i}, 'LineWidth',3); hold on;
end

%% plot cerberus 1 data
CERBERUS_OUTPUT_DATASET_FOLDER_PATH = [CERBERUS_OUTPUT_FOLDER_PATH,DATASET_NAME,'/'];
% check whether CERBERUS2_OUTPUT_DATASET_FOLDER_PATH is emppty
cerberus_traj_types =      {     'cerberus-wb',   'cerberus-wob',};
cerberus_traj_colors =     {'#0000FF','#00FF00'};
cerberus_traj_yaw_offset = { 0,        0,       };
total_types = size(cerberus_traj_types,2);
for i=1:total_types
    csv_file_full_name = strcat(CERBERUS_OUTPUT_DATASET_FOLDER_PATH,...
        cerberus_traj_types{i},'-',DATASET_NAME,'.csv');
    csv_file_full_name
    if isfile(csv_file_full_name)
            cerberus_data = readmatrix(csv_file_full_name);
    else 
        disp({csv_file_full_name, ' is not valid'})
    end
    plot3(cerberus_data(:,2),cerberus_data(:,3),cerberus_data(:,4),'Color',cerberus_traj_colors{i}, 'LineWidth',3); hold on;

end 


%% final adjustment to the figure
axis equal
view(0,90)
xlim(DATASET_X_RANGE)    
ylim(DATASET_Y_RANGE)
zlim(DATASET_Z_RANGE)
legend([traj_types, cerberus2_traj_types,cerberus_traj_types], 'Location','best')
