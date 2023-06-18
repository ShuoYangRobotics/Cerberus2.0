function [t0,time_list, pos_list, euler_list, vel_list] = parse_cerberus2_data(data, plot_start, plot_end)

% the input csv file should has format of time - pos - euer - velocity
% time should be nonzero UTC time 16XXXX.XXXX

% t0 is the first nonzero time

input_time = data(:,1);

nonzero_indices = find(input_time);
first_nonzero = input_time(nonzero_indices(1));
t0 = first_nonzero;
shift_time = input_time(nonzero_indices)-first_nonzero;

range_idx = shift_time > plot_start & shift_time < plot_end;
[~,idx,~] = unique(shift_time);
unique_idx = false(size(shift_time));
unique_idx(idx) = true;
select_idx = range_idx & unique_idx;

time_list = shift_time(select_idx);

pos_list = data(nonzero_indices,2:4);
pos_list = pos_list(select_idx,:);

% shift pos
pos_list(:,1) = pos_list(:,1)-pos_list(1,1); 
pos_list(:,2) = pos_list(:,2)-pos_list(1,2); 
pos_list(:,3) = pos_list(:,3)-pos_list(1,3);


euler_list = data(nonzero_indices,5:7);
euler_list = euler_list(select_idx,:);

vel_list = data(nonzero_indices,8:10);
vel_list = vel_list(select_idx,:);

end