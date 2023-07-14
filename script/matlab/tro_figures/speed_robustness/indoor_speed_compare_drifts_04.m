addpath("../../")
addpath(genpath("../matlab2tikz/"))

BAG_FOLDER_PATH = '/home/shuoyang/Documents/vilo_dev/vilo_ws/bags/';
CERBERUS_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus_output/'];
CERBERUS2_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus2_output/'];

             % name,       gt_yaw,        plot_time
zero4_DATASET_LIST =    {{'230620-risqh-trot-04-036-33square',                0, 99},...
                  {'230622-risqh-trot-04-040-33square',              0, 99},...
                  {'230622-risqh-trot-04-044-33square',              0, 99},...
                  ...
                  {'230622-risqh-trot-04-048-33square',              0, 99},...
                  {'230622-risqh-trot-04-052-33square',              0, 99},...
                  {'230625-risqh-flytrot-04-032-33square',              0, 99},...
                  ...
                  {'230625-risqh-flytrot-04-036-33square',              0, 99},...
                  {'230625-risqh-flytrot-04-040-33square',              0, 99},...
                  {'230625-risqh-standtrot-04-036-33square',              0, 99},...
                  ...
                  {'230625-risqh-standtrot-04-040-33square',              0, 99},...
                  {'230625-risqh-standtrot-04-044-33square',              0, 99},...
                  {'230625-risqh-standtrot-04-048-33square',              0, 99},...
                  };

ROW = 5;
    
figure(1);
set(gcf,'color','w');
total_cells = size(zero4_DATASET_LIST,2);

zero4_total_drifts = [];

t = tiledlayout(ceil(total_cells/ROW),ROW,'TileSpacing','Compact','Padding','Compact');
title('Experiments with speed 0.2m/s')

for item_idx = 1:size(zero4_DATASET_LIST,2)
    data_info = zero4_DATASET_LIST{item_idx};
    DATASET_NAME = data_info{1};
    CERBERUS_OUTPUT_DATASET_FOLDER_PATH = [CERBERUS_OUTPUT_FOLDER_PATH,DATASET_NAME,'/'];
    CERBERUS2_OUTPUT_DATASET_FOLDER_PATH = [CERBERUS2_OUTPUT_FOLDER_PATH,DATASET_NAME,'/'];


    num_methods = 6; % standard PO, 
    all_drift_list = [];   % Nx(num_datasets*num_methods+(num_datasets-1))
    all_color_maps = [];   %(num_datasets*num_methods+(num_datasets-1))x3 colormap for boxplot
    all_draw_labels = {};
    all_nan_indices = [];  % store index of nan lines, use for drawing separation lines

    plot_traj_colors = {};
    plot_traj_legends={};

    %% plot baseline
    % check whether CERBERUS2_OUTPUT_DATASET_FOLDER_PATH is emppty

    baseline_traj_types =  {     'gt',  'sipo',  'mipo',      'vio'};
    baseline_traj_colors = {'#0072BD','#D95319','#EDB120','#7E2F8E'};

    baseline_traj_legend =  {'Ground Truth',   'Standard PO', 'Multi-IMU PO',  'VINS-Fusion'};

    baseline_total_types = size(baseline_traj_types,2);

    plot_start = 0;
    plot_end = data_info{3}

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
    
    %% rotate GT
    yaw_offset = data_info{2};
    init_yaw = - yaw_offset/180*pi;
    R_yaw = [cos(init_yaw) sin(init_yaw) 0;
            -sin(init_yaw) cos(init_yaw) 0;
            0 0 1];
    baseline_traj_pos{1} = baseline_traj_pos{1}*R_yaw';
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
    cerberus2_traj_types =  {'vilo-m', 'vilo-tm-n'};
    cerberus2_traj_colors = {'#77AC30',   '#000000'};

    cerberus2_traj_legend =  {'Cerberus2-L', 'Cerberus2-T'};

    % traj_types = {'mipo','vilo-tm-n'};
    cerberus2_total_types = size(cerberus2_traj_types,2);

    plot_start = 0;
    plot_end = data_info{3}
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
    zero4_total_drifts = [zero4_total_drifts; drifts_list];
    nexttile;
    plot(interp_gt_traj_pos(:,1),interp_gt_traj_pos(:,2),'Color',baseline_traj_colors{1}, 'LineWidth',3); hold on;
    for i=2:baseline_total_types
        plot(interp_baseline_traj_pos{i-1}(:,1),interp_baseline_traj_pos{i-1}(:,2),'Color',baseline_traj_colors{i}, 'LineWidth',3); 
    end
    for i=1:cerberus1_total_types
        plot(interp_c1_traj_pos{i}(:,1),interp_c1_traj_pos{i}(:,2),'Color',cerberus_traj_colors{i}, 'LineWidth',3); 
    end
    for i=1:cerberus2_total_types
        plot(interp_c2_traj_pos{i}(:,1),interp_c2_traj_pos{i}(:,2),'Color',cerberus2_traj_colors{i}, 'LineWidth',3); 
    end

    %% final adjustment to the figure
    axis equal
    view(0,90)
    xlim([-1 4])
    ylim([-1 4])
end
legend([baseline_traj_legend cerberus_traj_legend,cerberus2_traj_legend], 'Location','Northwest')
