% add mobile gps process code
addpath('../mobile_gps_process/')
addpath(genpath('matlab2tikz/'))
addpath('../')

% prepare necessary file paths
BAG_FOLDER_PATH = '/home/rosie2/vilo_dev/vilo_ws/bags/';
CERBERUS_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus_output/'];
CERBERUS2_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus2_output/'];

%% only need to change these three usually
DATASET_FOLDER_NAME = '230304_wightman';
DATASET_NAME = '20230304_wightman_park_trot_bridge_loop';
DATASET_TIME = 417;

% DATASET_FOLDER_NAME = '230630_schenley';
% DATASET_NAME = '230630-schen-trot-08-038-tennis-to-overlook';
% DATASET_TIME = 530;

SAMPLE_RATE = 100.0;
GT_TIME_OFFSET = 100; % how much GT is longer than the robot dataset
GT_YAW_OFFSET = 0; % deg of gt

DATASET_X_RANGE = [-59 16];
DATASET_Y_RANGE = [-7 44];
DATASET_Z_RANGE = [-55 55];

%% if mobile exists
% process mobile data
mobile_gps_file_name = [BAG_FOLDER_PATH,'/',DATASET_FOLDER_NAME,'/',...
    DATASET_NAME,'.mat'];
has_mobile_gt = 0;
if isfile(mobile_gps_file_name)
    gps_position = position_filter(mobile_gps_file_name, DATASET_TIME+GT_TIME_OFFSET, GT_YAW_OFFSET,SAMPLE_RATE);
    has_mobile_gt = 1;
else
    has_mobile_gt = 0;
end
%%
gps_position = movmean(gps_position,75,1);
%% prepare figure
figure(1);clf

plot_didx = 10;
% plot gps_position as gt
traj_types =      {     'gt'};
traj_colors =     {'#0072BD'};

traj_legend =  {'Ground Truth'};
plot(gps_position(1:400:end,1),gps_position(1:400:end,2), 'Color',traj_colors{1}, 'LineWidth',3); hold on;

%% plot cerberus2
CERBERUS2_OUTPUT_DATASET_FOLDER_PATH = [CERBERUS2_OUTPUT_FOLDER_PATH,DATASET_NAME,'/'];
% check whether CERBERUS2_OUTPUT_DATASET_FOLDER_PATH is emppty

% look at src/utils/parameters.cpp for possible types
% traj_types = {'gt','mipo','sipo','vio','vilo-m','vilo-s'};
cerberus2_traj_types = {'sipo','mipo'};
cerberus2_traj_colors ={'#D95319','#EDB120'};

cerberus2_traj_legend =  {'Standard PO', 'Multi-IMU PO'};
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
 plot(traj_pos{i}(1:plot_didx:end,1),traj_pos{i}(1:plot_didx:end,2),'Color',cerberus2_traj_colors{i}, 'LineWidth',3); hold on;
end

%% final adjustment to the figure
axis equal
xlim(DATASET_X_RANGE)    
ylim(DATASET_Y_RANGE)
zlim(DATASET_Z_RANGE)
legend([traj_legend, cerberus2_traj_legend], 'Location','northwest')
set(gca,'Box','off');

matlab2tikz(strcat('outdoor_sipo_mipo_compare2.tex'), 'height', '\fheight', 'width', '\fwidth');