
BAG_FOLDER_PATH = '/home/shuoyang/Documents/vilo_dev/vilo_ws/bags/';
CERBERUS_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus_output/'];
CERBERUS2_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus2_output/'];

             % name,       gt_yaw,        plot_time
DATASET_LIST =    {{'230620-risqh-trot-05-036-33square',                0, 30},...
                  {'230620-risqh-trot-05-040-33square',              0, 30},...
                  {'230620-risqh-trot-05-044-33square',              0, 30},...
                  {'230620-risqh-trot-05-048-33square',   0, 30},...
                  {'230620-risqh-trot-05-052-33square',              0, 30}};

    
figure(1);clf
title('different speed')
total_cells = 0;
if rem(size(DATASET_LIST,2),2) == 1
    total_cells = size(DATASET_LIST,2) + 1;
else
    total_cells = size(DATASET_LIST,2);
end


for item_idx = 1:size(DATASET_LIST,2)
    data_info = DATASET_LIST{item_idx};
    DATASET_NAME = data_info{1};
    CERBERUS_OUTPUT_DATASET_FOLDER_PATH = [CERBERUS_OUTPUT_FOLDER_PATH,DATASET_NAME,'/'];
    CERBERUS2_OUTPUT_DATASET_FOLDER_PATH = [CERBERUS2_OUTPUT_FOLDER_PATH,DATASET_NAME,'/'];

    % look at src/utils/parameters.cpp for possible types
    cerberus2_traj_types =      {     'gt',   'mipo',  'vio',  'vilo-m', 'vilo-tm-n', 'vilo-tm-y'};
    cerberus2_traj_colors =     {'#0072BD','#D95319','#7E2F8E','#77AC30','#000000',   '#A00F00'};
    cerberus2_traj_yaw_offset = { data_info{2},        0,         0,   0,      0,             0};
    total_types = size(cerberus2_traj_types,2);
    
    plot_start = 0;
    plot_end = data_info{3}

    %% read data from cerberus 2
    traj_data = cell(1, total_types);
    for i=1:total_types
        csv_file_full_name = [CERBERUS2_OUTPUT_DATASET_FOLDER_PATH,...
            cerberus2_traj_types{i},'-',DATASET_NAME,'.csv'];
        csv_file_full_name
        if isfile(csv_file_full_name)
            traj_data{i} = readmatrix(csv_file_full_name);
        else 
            disp({csv_file_full_name, ' is not valid'})
        end
    end
    
    %
    traj_t0 = zeros(1, total_types);
    traj_time = cell(1, total_types);
    traj_pos = cell(1, total_types);
    traj_euler = cell(1, total_types);
    
    for i=1:total_types
        [traj_t0(i), traj_time{i}, traj_pos{i}, traj_euler{i}, ~] =...
            parse_cerberus2_data(traj_data{i}, plot_start, plot_end);
    end
    
    traj_t0 = traj_t0 - min(traj_t0);
    for i=1:total_types
    traj_time{i} = traj_time{i}+traj_t0(i);
    end
    
    subplot(total_cells/2,2,item_idx)
    for i=1:total_types
        init_yaw = cerberus2_traj_yaw_offset{i}/180*pi;
        R_yaw = [cos(init_yaw) sin(init_yaw) 0;
                -sin(init_yaw) cos(init_yaw) 0;
                 0 0 1];
        traj_pos{i} = traj_pos{i} * R_yaw;
        plot3(traj_pos{i}(:,1),traj_pos{i}(:,2),traj_pos{i}(:,3),'Color',cerberus2_traj_colors{i}, 'LineWidth',3); hold on;
    end
    % construct plot
    %% plot cerberus 1 data
    cerberus_traj_types =      {     'cerberus-wb',   'cerberus-wob',};
    cerberus_traj_colors =     {'#0000FF','#00FF00'};
    cerberus_traj_yaw_offset = { 0,        0,       };
    total_types = size(cerberus_traj_types,2);
    for i=1:total_types
        csv_file_full_name = strcat(CERBERUS_OUTPUT_DATASET_FOLDER_PATH,...
            cerberus_traj_types{i},'-',DATASET_NAME,'.csv');
        if isfile(csv_file_full_name)
                cerberus_data = readmatrix(csv_file_full_name);
        else 
            disp({csv_file_full_name, ' is not valid'})
        end
        plot3(cerberus_data(:,2),cerberus_data(:,3),cerberus_data(:,4),'Color',cerberus_traj_colors{i}, 'LineWidth',3); hold on;

    end 
    %% final legend and figure adjustment
    axis equal
    view(0,90)
    legend([cerberus2_traj_types  cerberus_traj_types], 'Location','best')
    xlim([-2 5])
    ylim([-1.5 4.5])
end