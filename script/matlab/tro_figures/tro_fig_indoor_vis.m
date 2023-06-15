addpath("../")
addpath(genpath("matlab2tikz/"))
% prepare necessary file paths
BAG_FOLDER_PATH = '/home/shuoyang/Documents/vilo_dev/vilo_ws/bags/';
CERBERUS_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus_output/'];
CERBERUS2_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus2_output/'];

%% only need to change these three usually
DATASET_FOLDER_NAME = '230525';
DATASET_NAME = '20230525_new_IMU_round';
DATASET_TIME = 26;

DATASET_X_RANGE = [-2.5 2.5];
DATASET_Y_RANGE = [-1.5 2];
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
 
traj_t0 = zeros(1, baseline_total_types);
traj_time = cell(1, baseline_total_types);
traj_pos = cell(1, baseline_total_types);
traj_euler = cell(1, baseline_total_types);

for i=1:baseline_total_types
    [traj_t0(i), traj_time{i}, traj_pos{i}, traj_euler{i}, ~] =...
        parse_cerberus2_data(traj_data{i}, plot_start, plot_end);
end

traj_t0 = traj_t0 - min(traj_t0);
for i=1:baseline_total_types
traj_time{i} = traj_time{i}+traj_t0(i);
end



for i=1:baseline_total_types
 plot3(traj_pos{i}(:,1),traj_pos{i}(:,2),traj_pos{i}(:,3),'Color',baseline_traj_colors{i}, 'LineWidth',3); hold on;
end

%% plot cerberus 1 data

cerberus_traj_types =      {  'cerberus-wob',};
cerberus_traj_legend =      {  'Cerberus',};
cerberus_traj_colors =     {'#00FF00'};
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

    [~, ~, cerberus_pos, ~, ~] =...
        parse_cerberus_data(cerberus_data, plot_start, plot_end);

    plot3(cerberus_pos(:,1),cerberus_pos(:,2),cerberus_pos(:,3),'Color',cerberus_traj_colors{i}, 'LineWidth',3); hold on;

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

%% final adjustment to the figure
axis equal
view(0,90)
xlim(DATASET_X_RANGE)    
ylim(DATASET_Y_RANGE)
zlim(DATASET_Z_RANGE)
legend([baseline_traj_legend cerberus_traj_legend,cerberus2_traj_legend], 'Location','best')

%% export to tikz
% cleanfigure;
matlab2tikz(strcat('tro_indoor_compare1.tex'), 'height', '\fheight', 'width', '\fwidth');