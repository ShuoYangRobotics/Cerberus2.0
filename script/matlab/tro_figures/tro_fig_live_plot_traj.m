% this is run after all data sequences are generated 
% first plot everything
% manually write the 6 methods 
figure(101);clf;
set(gcf,'Color','white');
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
p1_data_X = interp_gt_traj_pos(:,1);  p1_data_Y = interp_gt_traj_pos(:,2);  
p1 = plot(interp_gt_traj_pos(:,1),interp_gt_traj_pos(:,2), 'Color',gt_traj_colors{1}, 'LineWidth',3); hold on;
p1.XDataSource = 'p1_data_X'; p1.YDataSource = 'p1_data_Y';

p2_data_X = interp_baseline_traj_pos{1}(:,1);  p2_data_Y = interp_baseline_traj_pos{1}(:,2);  
p2 = plot(interp_baseline_traj_pos{1}(:,1),interp_baseline_traj_pos{1}(:,2),'Color',baseline_traj_colors{1}, 'LineWidth',3); 
p2.XDataSource = 'p2_data_X'; p2.YDataSource = 'p2_data_Y';

p3_data_X = interp_baseline_traj_pos{2}(:,1);  p3_data_Y = interp_baseline_traj_pos{2}(:,2);  
p3 = plot(interp_baseline_traj_pos{2}(:,1),interp_baseline_traj_pos{2}(:,2),'Color',baseline_traj_colors{2}, 'LineWidth',3); 
p3.XDataSource = 'p3_data_X'; p3.YDataSource = 'p3_data_Y';

p4_data_X = interp_baseline_traj_pos{3}(:,1);  p4_data_Y = interp_baseline_traj_pos{3}(:,2);  
p4 = plot(interp_baseline_traj_pos{3}(:,1),interp_baseline_traj_pos{3}(:,2),'Color',baseline_traj_colors{3}, 'LineWidth',3); 
p4.XDataSource = 'p4_data_X'; p4.YDataSource = 'p4_data_Y';

p5_data_X = interp_c1_traj_pos{1}(:,1);  p5_data_Y = interp_c1_traj_pos{1}(:,2);  
p5 = plot(interp_c1_traj_pos{1}(:,1),interp_c1_traj_pos{1}(:,2),'Color',cerberus_traj_colors{1}, 'LineWidth',3); 
p5.XDataSource = 'p5_data_X'; p5.YDataSource = 'p5_data_Y';

p6_data_X = interp_c2_traj_pos{1}(:,1);   p6_data_Y = interp_c2_traj_pos{1}(:,2);  
p6 = plot(interp_c2_traj_pos{1}(:,1),interp_c2_traj_pos{1}(:,2),'Color',cerberus2_traj_colors{1}, 'LineWidth',3); 
p6.XDataSource = 'p6_data_X'; p6.YDataSource = 'p6_data_Y';

p7_data_X = interp_c2_traj_pos{2}(:,1);  p7_data_Y = interp_c2_traj_pos{2}(:,2);  
p7 = plot(interp_c2_traj_pos{2}(:,1),interp_c2_traj_pos{2}(:,2),'Color',cerberus2_traj_colors{2}, 'LineWidth',3); 
p7.XDataSource = 'p7_data_X'; p7.YDataSource = 'p7_data_Y';

% all these sequences should be sync with interp_t_list

% final adjustment to the figure
axis equal
% view(0,90)
xlim(DATASET_X_RANGE)    
ylim(DATASET_Y_RANGE)
zlim(DATASET_Z_RANGE)
if has_mobile_gt == 1
    legend([traj_legend, baseline_traj_legend, cerberus_traj_legend,cerberus2_traj_legend], 'Location','northwest')
else
    legend([baseline_traj_legend cerberus_traj_legend,cerberus2_traj_legend], 'Location','northwest')
end


%%
total_viz_time = interp_t_list(end)-interp_t_list(1);
viz_rate = 30;
viz_dt = 1/viz_rate;
viz_end_time = 0.04;

r = rateControl(viz_rate);
reset(r)
v = VideoWriter([DATASET_NAME,'-viz-video']);
open(v);
while (1)
	time = r.TotalElapsedTime;
	fprintf('Time Elapsed: %f\n',time)
	waitfor(r);
    viz_end_time = interp_t_list(1)+time;
    p1_data_X = interp1(interp_t_list,interp_gt_traj_pos(:,1), interp_t_list(1):viz_dt:viz_end_time);
    p1_data_Y = interp1(interp_t_list,interp_gt_traj_pos(:,2), interp_t_list(1):viz_dt:viz_end_time);

    p2_data_X = interp1(interp_t_list,interp_baseline_traj_pos{1}(:,1), interp_t_list(1):viz_dt:viz_end_time);
    p2_data_Y = interp1(interp_t_list,interp_baseline_traj_pos{1}(:,2), interp_t_list(1):viz_dt:viz_end_time);

    p3_data_X = interp1(interp_t_list,interp_baseline_traj_pos{2}(:,1), interp_t_list(1):viz_dt:viz_end_time);
    p3_data_Y = interp1(interp_t_list,interp_baseline_traj_pos{2}(:,2), interp_t_list(1):viz_dt:viz_end_time);

    p4_data_X = interp1(interp_t_list,interp_baseline_traj_pos{3}(:,1), interp_t_list(1):viz_dt:viz_end_time);
    p4_data_Y = interp1(interp_t_list,interp_baseline_traj_pos{3}(:,2), interp_t_list(1):viz_dt:viz_end_time);

    p5_data_X = interp1(interp_t_list,interp_c1_traj_pos{1}(:,1), interp_t_list(1):viz_dt:viz_end_time);
    p5_data_Y = interp1(interp_t_list,interp_c1_traj_pos{1}(:,2), interp_t_list(1):viz_dt:viz_end_time);

    p6_data_X = interp1(interp_t_list,interp_c2_traj_pos{1}(:,1), interp_t_list(1):viz_dt:viz_end_time);
    p6_data_Y = interp1(interp_t_list,interp_c2_traj_pos{1}(:,2), interp_t_list(1):viz_dt:viz_end_time);

    p7_data_X = interp1(interp_t_list,interp_c2_traj_pos{2}(:,1), interp_t_list(1):viz_dt:viz_end_time);
    p7_data_Y = interp1(interp_t_list,interp_c2_traj_pos{2}(:,2), interp_t_list(1):viz_dt:viz_end_time);

    refreshdata
    frame = getframe(gcf);
    writeVideo(v,frame);
    if (time > total_viz_time)
        break;
    end
end
close(v);