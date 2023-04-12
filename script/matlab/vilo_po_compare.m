% plot vilo and po in matlab

bag_output_path = '../../bags/output/';

filetime = '2023-04-11_22-01-11';           % modify this
dataset_name = 'lab';       % modify this

plot_start = 0;
plot_end = 43;
%% read data
po_csv_file_full_name = strcat(bag_output_path,'po',filetime,'-',dataset_name);
po_data = readmatrix(po_csv_file_full_name);

vilo_csv_file_full_name = strcat(bag_output_path,'vilo',filetime,'-',dataset_name);
vilo_data = readmatrix(vilo_csv_file_full_name);

gt_csv_file_full_name = strcat(bag_output_path,'gt',filetime,'-',dataset_name);
gt_data = readmatrix(gt_csv_file_full_name);

%% 
[po_t0, po_time, po_pos, po_euler, ~] = parse_data(po_data, plot_start, plot_end);
[vilo_t0, vilo_time, vilo_pos, vilo_euler, ~] = parse_data(vilo_data, plot_start, plot_end);
[gt_t0, gt_time, gt_pos, gt_euler, ~] = parse_data(gt_data, plot_start, plot_end);

% shift times according to t0
t0_list = [po_t0 vilo_t0 gt_t0];
t0_list = t0_list - min(t0_list);
po_time = po_time+t0_list(1);
vilo_time = vilo_time+t0_list(2);
gt_time = gt_time+t0_list(3);

figure(1);clf
subplot(3,1,1)
p1 = plot3(po_pos(:,1),po_pos(:,2),po_pos(:,3)); hold on;
axis equal
p2 = plot3(vilo_pos(:,1),vilo_pos(:,2),vilo_pos(:,3)); hold on;
p3 = plot3(gt_pos(:,1),gt_pos(:,2),gt_pos(:,3)); hold on;
legend([p1 p2 p3], {'Multi-IMU PO', 'VIO', 'gt'})

subplot(3,1,2)
p1 = plot(po_time,po_euler(:,3)); hold on;
p2 = plot(vilo_time,vilo_euler(:,3)); hold on;
p3 = plot(gt_time,gt_euler(:,3));
legend([p1 p2 p3], {'Multi-IMU PO', 'VIO', 'gt'})