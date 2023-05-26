cerberus2_bag_output_path = '../../../../bags/cerberus2_output/multi/';

             % name,       gt_yaw,        plot_time
cerberus2_data_info_list = {{'20230517_risqh_02speed_mocap',                5, 80},...
                  {'20230517_risqh_04speed_mocap',              5, 44},...
                  {'20230517_risqh_06speed_mocap',              0, 33},...
                  {'20230517_risqh_08speed_mocap_more_turns',   0, 35},...
                  {'20230517_risqh_10speed_mocap',              0, 29}};

cerberus_bag_output_path = '../../../../bags/cerberus_output/';
% use the same data_info_list
    
figure(1);clf
title('different speed')
total_cells = 0;
if rem(size(cerberus2_data_info_list,2),2) == 1
    total_cells = size(cerberus2_data_info_list,2) + 1;
else
    total_cells = size(cerberus2_data_info_list,2);
end


for item_idx = 1:size(cerberus2_data_info_list,2)
    data_info = cerberus2_data_info_list{item_idx};
    dataset_name = data_info{1};       

    % look at src/utils/parameters.cpp for possible types
    cerberus2_traj_types =      {     'gt',   'mipo',  'vio',  'vilo-m', 'vilo-tm-n', 'vilo-tm-y'};
    cerberus2_traj_colors =     {'#0072BD','#D95319','#7E2F8E','#77AC30','#000000',   '#A00F00'};
    cerberus2_traj_yaw_offset = { data_info{2},        0,         0,   0,      0,             0};
    total_types = size(cerberus2_traj_types,2);
    
    plot_start = 0;
    plot_end = data_info{3};

    %% read data from cerberus 2
    traj_data = cell(1, total_types);
    for i=1:total_types
        csv_file_full_name = strcat(cerberus2_bag_output_path,cerberus2_traj_types{i},'-',dataset_name,'.csv');
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
        csv_file_full_name = strcat(cerberus_bag_output_path,cerberus_traj_types{i},'-',dataset_name,'.csv');
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