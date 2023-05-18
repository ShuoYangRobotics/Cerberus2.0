% plot vilo and po in matlab

bag_output_path = '../../bags/output/';

% dataset_name = '20230517_risqh_02speed_mocap';       % modify this
% dataset_name = '20230517_risqh_04speed_mocap';       % modify this
% dataset_name = '20230517_risqh_06speed_mocap';       % modify this
dataset_name = '20230517_risqh_08speed_mocap_more_turns';       % modify this
% dataset_name = '20230517_risqh_10speed_mocap';       % modify this

% look at src/utils/parameters.cpp for possible types
traj_types =      {     'gt',   'mipo',   'sipo',    'vio', 'vilo-m', 'vilo-s', 'vilo-tm-n'};
traj_colors =     {'#0072BD','#D95319','#EDB120','#7E2F8E','#77AC30','#4DBEEE',   '#000000'};
traj_yaw_offset = {        0,        0,         0,       0,        0,        0,           0};
total_types = size(traj_types,2);

plot_start = 0;
% plot_end = 80;
% plot_end = 44;
% plot_end = 33;
plot_end = 35;
% plot_end = 29;
%% read data
traj_data = cell(1, total_types);
for i=1:total_types
    csv_file_full_name = strcat(bag_output_path,traj_types{i},'-',dataset_name,'.csv');
    if isfile(csv_file_full_name)
        traj_data{i} = readmatrix(csv_file_full_name);
    else 
        disp({csv_file_full_name, ' is not valid'})
    end
end

%% 
traj_t0 = zeros(1, total_types);
traj_time = cell(1, total_types);
traj_pos = cell(1, total_types);
traj_euler = cell(1, total_types);

for i=1:total_types
    [traj_t0(i), traj_time{i}, traj_pos{i}, traj_euler{i}, ~] =...
        parse_data(traj_data{i}, plot_start, plot_end);
end

traj_t0 = traj_t0 - min(traj_t0);
for i=1:total_types
traj_time{i} = traj_time{i}+traj_t0(i);
end


figure(1);clf

for i=1:total_types
    init_yaw = traj_yaw_offset{i}/180*pi;
    R_yaw = [cos(init_yaw) sin(init_yaw) 0;
            -sin(init_yaw) cos(init_yaw) 0;
             0 0 1];
    traj_pos{i} = traj_pos{i} * R_yaw;
    plot3(traj_pos{i}(:,1),traj_pos{i}(:,2),traj_pos{i}(:,3),'Color',traj_colors{i}, 'LineWidth',3); hold on;
end
axis equal
view(0,90)
    legend(traj_types)

% subplot(2,1,2)
% if (has_gt == 1)
%     p1 = plot(gt_time,gt_euler(:,3)); hold on;
% end
% p2 = plot(po_time,po_euler(:,3)); hold on;
% p3 = plot(vio_time,vio_euler(:,3)); hold on;
% p4 = plot(vilo_time,vilo_euler(:,3)); hold on;
% if (has_gt == 1)
%     legend([p1 p2 p3 p4],{'gt', 'Multi-IMU PO', 'VIO', 'VILO'})
% else
%     legend([p2 p3 p4],{'Multi-IMU PO', 'VIO', 'VILO'})
% end