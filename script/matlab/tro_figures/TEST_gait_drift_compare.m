addpath("../")
addpath(genpath("matlab2tikz/"))
% prepare necessary file paths
BAG_FOLDER_PATH = '/home/shuoyang/Documents/vilo_dev/vilo_ws/bags/';
CERBERUS_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus_output/'];
CERBERUS2_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus2_output/'];

%% only need to change these three usually
% DATASET_NAME = '20230615-risqh-flyingtrot-06-04-square';
% DATASET_NAME = '20230615-risqh-standtrot-06-06-square-2';


% DATASET_NAME = '20230615-risqh-standtrot-04-06-square';
DATASET_NAME = '20230615-risqh-flyingtrot-04-04-square-more-stable';
% DATASET_NAME = '20230517_risqh_04speed_mocap';
% 
DATASET_TIME = 50;

DATASET_X_RANGE = [-2.5 3.5];
DATASET_Y_RANGE = [-1.5 3];
DATASET_Z_RANGE = [-2 2];


CERBERUS_OUTPUT_DATASET_FOLDER_PATH = [CERBERUS_OUTPUT_FOLDER_PATH,DATASET_NAME,'/'];
CERBERUS2_OUTPUT_DATASET_FOLDER_PATH = [CERBERUS2_OUTPUT_FOLDER_PATH,DATASET_NAME,'/'];
%% prepare figure
figure(1);clf

%% plot baseline
% check whether CERBERUS2_OUTPUT_DATASET_FOLDER_PATH is emppty

% look at src/utils/parameters.cpp for possible types
baseline_traj_types =  {     'gt',  'sipo',  'mipo',      'vio'};
baseline_traj_colors = {'#0072BD','#D95319','#EDB120','#7E2F8E'};

baseline_traj_legend =  {'Ground Truth',   'Standard PO', 'Multi-IMU PO',  'VINS-Fusion'};

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



for i=1:baseline_total_types
 plot3(baseline_traj_pos{i}(:,1),baseline_traj_pos{i}(:,2),baseline_traj_pos{i}(:,3),'Color',baseline_traj_colors{i}, 'LineWidth',3); hold on;
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
    plot3(c1_traj_pos{i}(:,1),c1_traj_pos{i}(:,2),c1_traj_pos{i}(:,3),'Color',cerberus_traj_colors{i}, 'LineWidth',3); hold on;

end 
%% plot cerberus2
% check whether CERBERUS2_OUTPUT_DATASET_FOLDER_PATH is emppty

% look at src/utils/parameters.cpp for possible types
% traj_types = {'gt','mipo','sipo','vio','vilo-m','vilo-s'};
cerberus2_traj_types =  {'vilo-m', 'vilo-tm-n'};
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



for i=1:cerberus2_total_types
 plot3(c2_traj_pos{i}(:,1),c2_traj_pos{i}(:,2),c2_traj_pos{i}(:,3),'Color',cerberus2_traj_colors{i}, 'LineWidth',3); hold on;
end

%% final adjustment to the figure
axis equal
view(0,90)
xlim(DATASET_X_RANGE)    
ylim(DATASET_Y_RANGE)
zlim(DATASET_Z_RANGE)
legend([baseline_traj_legend cerberus_traj_legend,cerberus2_traj_legend], 'Location','best')

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
dt = 0.05;


interp_baseline_traj_pos = cell(1, baseline_total_types-1);
interp_c1_traj_pos = cell(1, cerberus1_total_types);
interp_c2_traj_pos = cell(1, cerberus2_total_types);

interp_gt_traj_pos = interp1(baseline_traj_time{1},baseline_traj_pos{1},t0:dt:t1);

plot_traj_colors={};

for i=2:baseline_total_types
    interp_baseline_traj_pos{i-1} = interp1(baseline_traj_time{i},baseline_traj_pos{i},t0:dt:t1);
    plot_traj_colors = [plot_traj_colors baseline_traj_colors{i}];
end
for i=1:cerberus1_total_types
    interp_c1_traj_pos{i} = interp1(c1_traj_time{i},c1_traj_pos{i},t0:dt:t1);
    plot_traj_colors = [plot_traj_colors cerberus_traj_colors{i}];
end
for i=1:cerberus2_total_types
    interp_c2_traj_pos{i} = interp1(c2_traj_time{i},c2_traj_pos{i},t0:dt:t1);
    plot_traj_colors = [plot_traj_colors cerberus2_traj_colors{i}];
end

% interp_baseline_traj_pos{1} - ground truth
methods = [interp_baseline_traj_pos interp_c1_traj_pos interp_c2_traj_pos];

[ave_drifts,drifts_list] = drift_compare(interp_gt_traj_pos,methods);

%% test box plot
figure('Color', 'w');
clf
c = validatecolor(plot_traj_colors, 'multiple')

C = [c];  % this is the trick for coloring the boxes


% regular plot
boxplot(drifts_list*100, 'colors', C,'Symbol','','Whisker',0.5); % label only two categories

set(gca, 'XLim', [0 8], 'YLim', [-5 30]);

