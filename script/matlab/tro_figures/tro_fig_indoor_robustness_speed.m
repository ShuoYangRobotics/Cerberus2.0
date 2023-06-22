%% 
% In this script
% we get three dataset results, each uses one gait
% then we compare all of their drifts, plot a box plot

addpath("../")
addpath(genpath("matlab2tikz/"))
% prepare necessary file paths
BAG_FOLDER_PATH = '/home/shuoyang/Documents/vilo_dev/vilo_ws/bags/';
CERBERUS_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus_output/'];
CERBERUS2_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus2_output/'];

%% prepare list of datasets
% for future experiments, should just need to modify these three items
DATASET_LIST = {'230620-risqh-trot-02-036-33square',...
                '230620-risqh-trot-04-036-33square',...
                '230620-risqh-trot-06-036-33square',...
                '230620-risqh-trot-08-036-33square',...
                '230620-risqh-trot-10-036-33square'};
DATASET_LABEL_LIST = {'0.2m/s',...
                      '0.4m/s',...
                      '0.6m/s',...
                      '0.8m/s',...
                      '1.0m/s'};
DATASET_TIME_LIST = {71,34,26, 25,25};


%% iterate through DATASET_LIST
num_datasets = length(DATASET_LIST);
num_methods = 5; % standard PO, 
all_drift_list = [];   % Nx(num_datasets*num_methods+(num_datasets-1))
all_color_maps = [];   %(num_datasets*num_methods+(num_datasets-1))x3 colormap for boxplot
all_draw_labels = {};
all_nan_indices = [];  % store index of nan lines, use for drawing separation lines

plot_traj_colors = {};
plot_traj_legends={};

for idx_dataset=1:num_datasets
    %% calculate drift for one 
    DATASET_NAME = DATASET_LIST{idx_dataset};
    
    DATASET_TIME = DATASET_TIME_LIST{idx_dataset};
    
    DATASET_X_RANGE = [-2.5 3.5];
    DATASET_Y_RANGE = [-1.5 3];
    DATASET_Z_RANGE = [-2 2];
    
    
    CERBERUS_OUTPUT_DATASET_FOLDER_PATH = [CERBERUS_OUTPUT_FOLDER_PATH,DATASET_NAME,'/'];
    CERBERUS2_OUTPUT_DATASET_FOLDER_PATH = [CERBERUS2_OUTPUT_FOLDER_PATH,DATASET_NAME,'/'];

    %% prepare baseline
    % check whether CERBERUS2_OUTPUT_DATASET_FOLDER_PATH is emppty
    
    % look at src/utils/parameters.cpp for possible types
    baseline_traj_types =  {     'gt',  'mipo',      'vio'};
    baseline_traj_colors = {'#0072BD','#EDB120','#7E2F8E'};
    
    baseline_traj_legend =  {'Ground Truth',   'Multi-IMU PO',  'VINS-Fusion'};
    
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
    
    
    
    % for i=1:baseline_total_types
    %  plot3(baseline_traj_pos{i}(:,1),baseline_traj_pos{i}(:,2),baseline_traj_pos{i}(:,3),'Color',baseline_traj_colors{i}, 'LineWidth',3); hold on;
    % end
    
    %% prepare cerberus 1 data
    
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
    %     plot3(c1_traj_pos{i}(:,1),c1_traj_pos{i}(:,2),c1_traj_pos{i}(:,3),'Color',cerberus_traj_colors{i}, 'LineWidth',3); hold on;
    
    end 
    %% prepare cerberus2
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
    
    
    interp_baseline_traj_pos = cell(1, baseline_total_types-1);
    interp_c1_traj_pos = cell(1, cerberus1_total_types);
    interp_c2_traj_pos = cell(1, cerberus2_total_types);
    
    interp_gt_traj_pos = interp1(baseline_traj_time{1},baseline_traj_pos{1},interp_t_list);
    
    plot_traj_colors={};
    plot_traj_legends={};
    
    for i=2:baseline_total_types
        interp_baseline_traj_pos{i-1} = interp1(baseline_traj_time{i},baseline_traj_pos{i},interp_t_list);
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
    
    % interp_baseline_traj_pos{1} - ground truth
    methods = [interp_baseline_traj_pos interp_c1_traj_pos interp_c2_traj_pos];
    
    [ave_drifts,drifts_list] = drift_compare(interp_gt_traj_pos,methods);
    
    %% put drift_list and color map to overall data structure
    c = validatecolor(plot_traj_colors, 'multiple');
    all_color_maps = [all_color_maps; c];
    all_drift_list = [all_drift_list drifts_list];
    item_per_group = size(drifts_list,2);
    all_draw_labels = [all_draw_labels {'','',DATASET_LABEL_LIST{idx_dataset},'',''}];
    if idx_dataset<num_datasets
        all_drift_list = [all_drift_list NaN*drifts_list(:,1)];
        all_color_maps =[all_color_maps; ones(1,3)];
        all_nan_indices = [all_nan_indices (item_per_group+1)*idx_dataset];
        all_draw_labels = [all_draw_labels {''}];
    end
end
%% test box plot
figure(4);
set(gcf,'Color', 'w');
clf
% regular plot
draw_y_lim_low = -1
draw_y_lim_high = 5;
boxplot(all_drift_list*100, 'colors', all_color_maps,...
    'labels', all_draw_labels,...
    'Symbol','','Whisker',0.5,'LabelOrientation','horizontal'); % label only two categories
hold on; 
set(gca, 'XLim', [0 size(all_drift_list,2)+1], 'YLim', [draw_y_lim_low draw_y_lim_high]);
for idx=1:length(all_nan_indices)
    y1=get(gca,'ylim');
    plot([all_nan_indices(idx) all_nan_indices(idx)],y1,'--k')
end
xtickangle(0)
ylabel('Drift (%)');

% create fake lines in order to add legend hands
c = validatecolor(plot_traj_colors, 'multiple');
fake_line_handles = [];
for ii = 1:length(plot_traj_colors)
    p_handle = plot(-5,-5,'color', c(ii,:), 'LineWidth', 4);

    fake_line_handles = [fake_line_handles p_handle];
end
legend(fake_line_handles,plot_traj_legends)


%%
matlab2tikz(strcat('tro_indoor_robustness_speed.tex'), 'height', '\fheight', 'width', '\fwidth');