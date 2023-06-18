function [ave_drifts,drifts_list] = drift_compare(gt_pos,method_pos_lists)
%DRIFT_COMPARE calculate drifts of methods
%   gt_pos is a Nx3 matrix contains ground truth pos traj
%   method_pos_lists is a cell array with M items, each is a Nx3 matrix

% ave_drifts is Mx1 matrix, drifts_list is NxM list
num_methods = size(method_pos_lists,2);
num_traj = size(gt_pos,1);

drifts_list = zeros(num_traj,num_methods);

% calculate travel distance
gt_pos_xy = gt_pos(:,1:2);
gt_pos_xy_shift = circshift(gt_pos_xy,1,1); gt_pos_xy_shift(1,:) = zeros(1,2);
diff = gt_pos_xy-gt_pos_xy_shift;
displacement = vecnorm(diff')';

travel_distance = cumsum(displacement);

for i=1:num_methods
    pos_error = gt_pos - method_pos_lists{i};
    pos_error_norm = vecnorm(pos_error')';
    idx = pos_error_norm > travel_distance;
    pos_error_norm(idx) = 0;

    drifts_list(:,i) = pos_error_norm./travel_distance;
    drifts_list(1:floor(end/2),i) = 0;
end
ave_drifts = mean(drifts_list(floor(end/2):end,:),1);
end