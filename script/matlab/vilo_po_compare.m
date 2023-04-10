% plot vilo and po in matlab

bag_output_path = '../../bags/output/';

filetime = '2023-04-10_17-07-26';           % modify this
dataset_name = 'lab';       % modify this

po_csv_file_full_name = strcat(bag_output_path,'po',filetime,'-',dataset_name);
po_data = readmatrix(po_csv_file_full_name);


vilo_csv_file_full_name = strcat(bag_output_path,'vilo',filetime,'-',dataset_name);
vilo_data = readmatrix(vilo_csv_file_full_name);

gt_csv_file_full_name = strcat(bag_output_path,'gt',filetime,'-',dataset_name);
gt_data = readmatrix(gt_csv_file_full_name);

%% 
po_time = po_data(:,1);
po_pos = po_data(:,2:4);

vilo_time = vilo_data(:,1);
vilo_pos = vilo_data(:,2:4);

gt_time = gt_data(:,1);
gt_pos = gt_data(:,2:4); gt_pos(:,1) = gt_pos(:,1)-gt_pos(1,1); gt_pos(:,2) = gt_pos(:,2)-gt_pos(1,2); gt_pos(:,3) = gt_pos(:,3)-gt_pos(1,3);

figure(1);clf
p1 = plot3(po_pos(:,1),po_pos(:,2),po_pos(:,3)); hold on;
axis equal
p2 = plot3(vilo_pos(:,1),vilo_pos(:,2),vilo_pos(:,3)); hold on;
p3 = plot3(gt_pos(:,1),gt_pos(:,2),gt_pos(:,3)); hold on;

legend([p1 p2 p3], {'Multi-IMU PO', 'VIO', 'gt'})