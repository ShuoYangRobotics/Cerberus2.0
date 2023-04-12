% plot vilo and po in matlab

bag_output_path = '../../bags/output/';

po_type ='mipo';
vio_type = 'vio';
vilo_type = 'vilo';
dataset_name = 'wightman_park_trot_bridge_loop';       % modify this
has_gt = 1;

plot_start = 0;
plot_end = 41;
%% read data
po_csv_file_full_name = strcat(bag_output_path,po_type,'-',dataset_name,'.csv');
if isfile(po_csv_file_full_name)
    po_data = readmatrix(po_csv_file_full_name);
end

vilo_csv_file_full_name = strcat(bag_output_path,vilo_type,'-',dataset_name,'.csv');
if isfile(vilo_csv_file_full_name)
    vilo_data = readmatrix(vilo_csv_file_full_name);
end

vio_csv_file_full_name = strcat(bag_output_path,vio_type,'-',dataset_name,'.csv');
if isfile(vio_csv_file_full_name)
    vio_data = readmatrix(vio_csv_file_full_name);
end

if (has_gt == 1)
    gt_csv_file_full_name = strcat(bag_output_path,'gt','-',dataset_name,'.csv');
    if isfile(gt_csv_file_full_name)
        gt_data = readmatrix(gt_csv_file_full_name);
    end
end

%% 
[po_t0, po_time, po_pos, po_euler, ~] = parse_data(po_data, plot_start, plot_end);
[vilo_t0, vilo_time, vilo_pos, vilo_euler, ~] = parse_data(vilo_data, plot_start, plot_end);
[vio_t0, vio_time, vio_pos, vio_euler, ~] = parse_data(vio_data, plot_start, plot_end);
if (has_gt == 1)
    [gt_t0, gt_time, gt_pos, gt_euler, ~] = parse_data(gt_data, plot_start, plot_end);
end

% shift times according to t0
if (has_gt == 1)
    t0_list = [po_t0 vilo_t0 vio_t0 gt_t0];
else
    t0_list = [po_t0 vilo_t0 vio_t0];
end

t0_list = t0_list - min(t0_list);
po_time = po_time+t0_list(1);
vilo_time = vilo_time+t0_list(2);
vio_time = vio_time+t0_list(3);

if (has_gt == 1)
    gt_time = gt_time+t0_list(4);
end

figure(1);clf
subplot(2,1,1)
if (has_gt == 1)
    p1 = plot3(gt_pos(:,1),gt_pos(:,2),gt_pos(:,3)); hold on;
end
p2 = plot3(po_pos(:,1),po_pos(:,2),po_pos(:,3)); hold on;
p3 = plot3(vio_pos(:,1),vio_pos(:,2),vio_pos(:,3)); hold on;
p4 = plot3(vilo_pos(:,1),vilo_pos(:,2),vilo_pos(:,3)); hold on;
axis equal
view(0,90)
if (has_gt == 1)
    legend([p1 p2 p3 p4],{'gt', 'Multi-IMU PO', 'VIO', 'VILO'})
else
    legend([p2 p3 p4],{'Multi-IMU PO', 'VIO', 'VILO'})
end

subplot(2,1,2)
if (has_gt == 1)
    p1 = plot(gt_time,gt_euler(:,3)); hold on;
end
p2 = plot(po_time,po_euler(:,3)); hold on;
p3 = plot(vio_time,vio_euler(:,3)); hold on;
p4 = plot(vilo_time,vilo_euler(:,3)); hold on;
if (has_gt == 1)
    legend([p1 p2 p3 p4],{'gt', 'Multi-IMU PO', 'VIO', 'VILO'})
else
    legend([p2 p3 p4],{'Multi-IMU PO', 'VIO', 'VILO'})
end