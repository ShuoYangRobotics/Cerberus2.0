% add mobile gps process code
addpath('../mobile_gps_process/')
addpath('../')

% prepare necessary file paths
BAG_FOLDER_PATH = '/home/rosie2/vilo_dev/vilo_ws/bags/';
CERBERUS_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus_output/'];
CERBERUS2_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus2_output/'];

%% only need to change these three usually
DATASET_FOLDER_NAME = '230628_mill19';
% DATASET_NAME = '230628-mil19-trot-07-039-wild1515-footIMU-fail-in-middle';
DATASET_NAME = '230628-mil19-trot-07-039-wild1445';
DATASET_TIME = 410;
SAMPLE_RATE = 100.0;
GT_TIME_OFFSET = 100; % how much GT is longer than the robot dataset
GT_YAW_OFFSET = -2; % deg of gt


% DATASET_NAME = '230630-schen-trot-08-038-tennis-to-overlook';
% DATASET_TIME = 530;
% GT_YAW_OFFSET = 2; % deg of gt
% 
% DATASET_NAME = '230630-schen-trot-08-038-loop-tennis';
% DATASET_TIME = 280;
% GT_YAW_OFFSET = -17; % deg of gt

DATASET_X_RANGE = [-10 160];
DATASET_Y_RANGE = [-10 180];
% DATASET_Z_RANGE = [-115 115];

%% if mobile exists
% process mobile data
mobile_gps_file_name = [BAG_FOLDER_PATH,'/',DATASET_FOLDER_NAME,'/',...
    DATASET_NAME,'.mat'];
save_mobile_gps_file_name = [BAG_FOLDER_PATH,'/',DATASET_FOLDER_NAME,'/',...
    DATASET_NAME,'_filter_gps.mat'];
has_mobile_gt = 0;
if isfile(mobile_gps_file_name)
    %%
    gps_position = position_filter(mobile_gps_file_name, DATASET_TIME+GT_TIME_OFFSET, GT_YAW_OFFSET,SAMPLE_RATE);
    gps_position = movmean(gps_position,15,1);
    % save (save_mobile_gps_file_name, 'gps_position');
    %%
    % load (save_mobile_gps_file_name)
    has_mobile_gt = 1;
else
    has_mobile_gt = 0;
end
%% prepare figure
figure(1);clf

% plot gps_position as gt
traj_types =      {     'gt'};
traj_colors =     {'#0072BD'};
traj_legend =  {'Ground Truth'};

if has_mobile_gt == 1
    plot(gps_position(:,1),gps_position(:,2), 'Color',traj_colors{1}, 'LineWidth',3); hold on;
end
%% plot baseline
% check whether CERBERUS2_OUTPUT_DATASET_FOLDER_PATH is emppty

CERBERUS_OUTPUT_DATASET_FOLDER_PATH = [CERBERUS_OUTPUT_FOLDER_PATH,DATASET_NAME,'/'];
CERBERUS2_OUTPUT_DATASET_FOLDER_PATH = [CERBERUS2_OUTPUT_FOLDER_PATH,DATASET_NAME,'/'];

baseline_traj_types =  {  'sipo',  'mipo',      'vio'};
baseline_traj_colors = {'#D95319','#EDB120','#7E2F8E'};

baseline_traj_legend =  {   'Standard PO', 'Multi-IMU PO',  'VINS-Fusion'};

baseline_total_types = size(baseline_traj_types,2);

plot_start = 0;
plot_end = DATASET_TIME;
% read data
traj_data = cell(1, baseline_total_types);
for i=1:baseline_total_types
    csv_file_full_name = [CERBERUS2_OUTPUT_DATASET_FOLDER_PATH,...
        baseline_traj_types{i},'-',DATASET_NAME,'.csv'];
    if isfile(csv_file_full_name)
        csv_file_full_name
        traj_data{i} = readmatrix(csv_file_full_name);
    else 
        csv_file_full_name
        disp({csv_file_full_name, ' is not valid'})
    end
end
 
baseline_traj_t0 = zeros(1, baseline_total_types);
baseline_traj_time = cell(1, baseline_total_types);
baseline_traj_pos = cell(1, baseline_total_types);
baseline_traj_euler = cell(1, baseline_total_types);

for i=1:baseline_total_types
    [baseline_traj_t0(i), baseline_traj_time{i}, baseline_traj_pos{i}, baseline_traj_euler{i}, ~] =...
        parse_cerberus2_data(traj_data{i}, plot_start, plot_end);
end

% baselin_traj_t0 = baselin_traj_t0 - min(baselin_traj_t0);
for i=1:baseline_total_types
baseline_traj_time{i} = baseline_traj_time{i}+baseline_traj_t0(i);
end



%% plot cerberus 1 data

cerberus_traj_types =      {  'cerberus-wob',};
cerberus_traj_legend =      {  'Cerberus',};
cerberus_traj_colors =     {'#00FF00'};
cerberus_traj_yaw_offset = { 0,        0,       };
cerberus1_total_types = size(cerberus_traj_types,2);

c1_traj_t0 = zeros(1, cerberus1_total_types);
c1_traj_time = cell(1, cerberus1_total_types);
c1_traj_pos = cell(1, cerberus1_total_types);
c1_traj_euler = cell(1, cerberus1_total_types);

for i=1:cerberus1_total_types
    csv_file_full_name = strcat(CERBERUS_OUTPUT_DATASET_FOLDER_PATH,...
        cerberus_traj_types{i},'-',DATASET_NAME,'.csv');
    csv_file_full_name
    if isfile(csv_file_full_name)
            cerberus_data = readmatrix(csv_file_full_name);
    else 
        disp({csv_file_full_name, ' is not valid'})
    end

    [c1_traj_t0(i), c1_traj_time{i}, c1_traj_pos{i}, c1_traj_euler{i}, ~] =...
        parse_cerberus_data(cerberus_data, plot_start, plot_end);

    c1_traj_time{i} = c1_traj_time{i}+c1_traj_t0(i);

end 
%% plot cerberus2
% check whether CERBERUS2_OUTPUT_DATASET_FOLDER_PATH is emppty

% look at src/utils/parameters.cpp for possible types
% traj_types = {'gt','mipo','sipo','vio','vilo-m','vilo-s'};
cerberus2_traj_types =  {'vilo-m','vilo-tm-n'};
cerberus2_traj_colors = {'#77AC30',   '#000000'};

cerberus2_traj_legend =  {'Cerberus2-L', 'Cerberus2-T'};

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
 
c2_traj_t0 = zeros(1, cerberus2_total_types);
c2_traj_time = cell(1, cerberus2_total_types);
c2_traj_pos = cell(1, cerberus2_total_types);
c2_traj_euler = cell(1, cerberus2_total_types);

for i=1:cerberus2_total_types
    i
    [c2_traj_t0(i), c2_traj_time{i}, c2_traj_pos{i}, c2_traj_euler{i}, ~] =...
        parse_cerberus2_data(traj_data{i}, plot_start, plot_end);
end

% c2_traj_t0 = c2_traj_t0 - min(c2_traj_t0);
for i=1:cerberus2_total_types
c2_traj_time{i} = c2_traj_time{i}+c2_traj_t0(i);
end

%% drift 
%% get min and max of times 
start_time_list = [];
end_time_list = [];

for i=1:baseline_total_types
    start_time_list = [start_time_list baseline_traj_time{i}(1)];
    end_time_list = [end_time_list baseline_traj_time{i}(end)];
end
for i=1:cerberus1_total_types
    start_time_list = [start_time_list c1_traj_time{i}(1)];
    end_time_list = [end_time_list c1_traj_time{i}(end)];
end
for i=1:cerberus2_total_types
    start_time_list = [start_time_list c2_traj_time{i}(1)];
    end_time_list = [end_time_list c2_traj_time{i}(end)];
end

%% interpolate all trajectories so we can compare drift
t0 = max(start_time_list);
t1 = min(end_time_list);
interp_t_list = linspace(t0,t1,500);
start_time_list - t0
end_time_list - t1


interp_baseline_traj_pos = cell(1, baseline_total_types);
interp_c1_traj_pos = cell(1, cerberus1_total_types);
interp_c2_traj_pos = cell(1, cerberus2_total_types);


plot_traj_colors={};
plot_traj_legends={};

for i=1:baseline_total_types
    interp_baseline_traj_pos{i} = interp1(baseline_traj_time{i},baseline_traj_pos{i},interp_t_list);
    plot_traj_colors = [plot_traj_colors baseline_traj_colors{i}];
    plot_traj_legends = [plot_traj_legends baseline_traj_legend{i}];
end
for i=1:cerberus1_total_types
    interp_c1_traj_pos{i} = interp1(c1_traj_time{i},c1_traj_pos{i},interp_t_list);
    plot_traj_colors = [plot_traj_colors cerberus_traj_colors{i}];
    plot_traj_legends = [plot_traj_legends cerberus_traj_legend{i}];
end
for i=1:cerberus2_total_types
    interp_c2_traj_pos{i} = interp1(c2_traj_time{i},c2_traj_pos{i},interp_t_list);
    plot_traj_colors = [plot_traj_colors cerberus2_traj_colors{i}];
    plot_traj_legends = [plot_traj_legends cerberus2_traj_legend{i}];
end


for i=1:baseline_total_types
    plot(interp_baseline_traj_pos{i}(:,1),interp_baseline_traj_pos{i}(:,2),'Color',baseline_traj_colors{i}, 'LineWidth',3); 
end
for i=1:cerberus1_total_types
    plot(interp_c1_traj_pos{i}(:,1),interp_c1_traj_pos{i}(:,2),'Color',cerberus_traj_colors{i}, 'LineWidth',3); 
end
for i=1:cerberus2_total_types
    plot(interp_c2_traj_pos{i}(:,1),interp_c2_traj_pos{i}(:,2),'Color',cerberus2_traj_colors{i}, 'LineWidth',3); 
end

%% final adjustment to the figure
axis equal
% view(0,90)
xlim(DATASET_X_RANGE)    
ylim(DATASET_Y_RANGE)
zlim(DATASET_Z_RANGE)
if has_mobile_gt == 1
    legend([traj_legend, baseline_traj_legend, cerberus_traj_legend,cerberus2_traj_legend], 'Location','best')
else
    legend([baseline_traj_legend cerberus_traj_legend,cerberus2_traj_legend], 'Location','best')
end
