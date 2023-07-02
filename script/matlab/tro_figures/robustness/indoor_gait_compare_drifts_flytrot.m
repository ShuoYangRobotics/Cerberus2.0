
BAG_FOLDER_PATH = '/home/rosie2/vilo_dev/vilo_ws/bags/';
CERBERUS_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus_output/'];
CERBERUS2_OUTPUT_FOLDER_PATH = [BAG_FOLDER_PATH,'cerberus2_output/'];

             % name,       gt_yaw,        plot_time
FLY_DATASET_LIST =    {{'230625-risqh-flytrot-02-032-33square',                0, 99},...
                  {'230625-risqh-flytrot-02-036-33square',              0, 99},...
                  {'230625-risqh-flytrot-02-040-33square',              0, 99},...
                  ...
                  {'230625-risqh-flytrot-04-032-33square',              0, 99},...
                  {'230625-risqh-flytrot-04-036-33square',              0, 99},...
                  {'230625-risqh-flytrot-04-040-33square',              0, 99},...
                  ...
                  {'230625-risqh-flytrot-06-032-33square',              0, 99},...
                  {'230625-risqh-flytrot-06-036-33square',              0, 99},...
                  {'230625-risqh-flytrot-06-040-33square',              0, 99},...
                  ...
                  {'230625-risqh-flytrot-08-032-33square',              0, 99},...
                  {'230625-risqh-flytrot-08-036-33square',              0, 99},...
                  {'230625-risqh-flytrot-08-040-33square',              0, 99},...
                  ...
                  {'230625-risqh-flytrot-10-032-33square',              0, 99},...
                  {'230625-risqh-flytrot-10-036-33square',              0, 99},...
                  {'230625-risqh-flytrot-12-036-33square',              0, 99}
                  };

ROW = 5;
    
figure(1);
title('different speed, gait time')
set(gcf,'color','w');
total_cells = size(FLY_DATASET_LIST,2);


t = tiledlayout(ceil(total_cells/ROW),ROW,'TileSpacing','Compact','Padding','Compact');

for item_idx = 1:size(FLY_DATASET_LIST,2)
    data_info = FLY_DATASET_LIST{item_idx};
    DATASET_NAME = data_info{1};
    CERBERUS_OUTPUT_DATASET_FOLDER_PATH = [CERBERUS_OUTPUT_FOLDER_PATH,DATASET_NAME,'/'];
    CERBERUS2_OUTPUT_DATASET_FOLDER_PATH = [CERBERUS2_OUTPUT_FOLDER_PATH,DATASET_NAME,'/'];

    % look at src/utils/parameters.cpp for possible types
    cerberus2_traj_types =      {     'gt',   'sipo', 'mipo', 'vio',  'vilo-m', 'vilo-tm-n'};
    cerberus2_traj_colors =     {'#0072BD','#D95319','#EDB120','#7E2F8E','#77AC30','#000000'};
    cerberus2_traj_legends =     {'Ground Truth',   'Standard PO', 'Multi-IMU PO',  'VINS-Fusion', 'Cerberus2-L', 'Cerberus2-T'};
    cerberus2_traj_yaw_offset = { data_info{2},        0,         0,   0,      0,   0};
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
    
    % subplot(total_cells/2,2,item_idx)
    nexttile;
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
    cerberus_traj_types =      {'cerberus-wob',};
    cerberus_traj_legends =      {'Cerberus',};
    cerberus_traj_colors =     {'#00FF00'};
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
    xlim([-1 4])
    ylim([-1 4])
end
legend([cerberus2_traj_legends  cerberus_traj_legends], 'Location','best')