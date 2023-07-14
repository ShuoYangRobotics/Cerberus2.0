indoor_gait_compare_drifts_flytrot;
indoor_gait_compare_drifts_trot;
indoor_gait_compare_drifts_standtrot;
%%
DATASET_LABEL_LIST = {'Standing Trot',...
                      'Trot',...
                      'Flying Trot'};

num_gaits = 3;
num_methods = 6; % standard PO, 
all_drift_list = [];   % Nx(num_datasets*num_methods+(num_datasets-1))
all_color_maps = [];   %(num_datasets*num_methods+(num_datasets-1))x3 colormap for boxplot
all_draw_labels = {};
all_nan_indices = [];  % store index of nan lines, use for drawing separation lines


total = 17000;
%
if size(standtrot_total_drifts,1)<total
    padding = total - size(standtrot_total_drifts,1);
    height = size(standtrot_total_drifts,2);
    all_drift_list = [all_drift_list [standtrot_total_drifts; nan*ones(padding,height)]];
else
    all_drift_list = [all_drift_list standtrot_total_drifts(1:total,:)];
end
all_drift_list = [all_drift_list NaN*ones(total,1)];
if size(trot_total_drifts,1)<total
    padding = total - size(trot_total_drifts,1);
    height = size(trot_total_drifts,2);
    all_drift_list = [all_drift_list [trot_total_drifts; nan*ones(padding,height)]];
else
    all_drift_list = [all_drift_list trot_total_drifts(1:total,:)];
end
all_drift_list = [all_drift_list NaN*ones(total,1)];
if size(flytrot_total_drifts,1)<total
    padding = total - size(flytrot_total_drifts,1);
    height = size(flytrot_total_drifts,2);
    all_drift_list = [all_drift_list [flytrot_total_drifts; nan*ones(padding,height)]];
else
    all_drift_list = [all_drift_list flytrot_total_drifts(1:total,:)];
end

for idx_dataset=1:num_gaits
    % put drift_list and color map to overall data structure
    c = validatecolor(plot_traj_colors, 'multiple');
    all_color_maps = [all_color_maps; c];
    item_per_group = num_methods;
    all_draw_labels = [all_draw_labels {'','',DATASET_LABEL_LIST{idx_dataset},'','',''}];
    if idx_dataset<num_gaits
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
draw_y_lim_low = -1;
draw_y_lim_high = 15;
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
title('Median Drifts at Different Gaits')
