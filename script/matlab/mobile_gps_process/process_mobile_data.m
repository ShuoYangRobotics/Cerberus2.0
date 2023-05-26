function [sensor_data_orient, gps_valid, sensor_data_imu_gps] = process_mobile_data(mobile_data_file, sample_dt, total_time)
%PROCESS_MOBILE_DATA input a mobile data file, output a data structure.
%   sample_rate should be 1/100 for iphone data

% at the beginning assume gps is not valid
gps_valid = 0;

mobile_data = load(mobile_data_file);

% rename tables 
mobile_data.Acceleration = renamevars(mobile_data.Acceleration,["X","Y","Z"],["AccX","AccY","AccZ"]);
mobile_data.AngularVelocity = renamevars(mobile_data.AngularVelocity,["X","Y","Z"],["GyroX","GyroY","GyroZ"]);
mobile_data.MagneticField = renamevars(mobile_data.MagneticField,["X","Y","Z"],["MagX","MagY","MagZ"]);
mobile_data.Orientation = renamevars(mobile_data.Orientation,["X","Y","Z"],["OrientX","OrientY","OrientZ"]);

% how to deal with GPS position? 
% num of GPS must larger than a certain value 

% synchronize timetables 
sensor_data = synchronize(mobile_data.Acceleration,...
    mobile_data.AngularVelocity,...
    mobile_data.MagneticField,...
    mobile_data.Orientation,...
    'regular','linear','TimeStep',seconds(sample_dt));

% convert to NED here
sensor_data_tmp = sensor_data; % copy
sensor_data_tmp.AccX = -movmean(sensor_data.AccY,1,1);
sensor_data_tmp.AccY = -movmean(sensor_data.AccX,1,1);
sensor_data_tmp.AccZ = movmean(sensor_data.AccZ,1,1);

sensor_data_tmp.GyroX = sensor_data.GyroY;
sensor_data_tmp.GyroY = sensor_data.GyroX ;
sensor_data_tmp.GyroZ = -sensor_data.GyroZ;

sensor_data_tmp.MagX = sensor_data.MagY;
sensor_data_tmp.MagY = sensor_data.MagX;
sensor_data_tmp.MagZ = -sensor_data.MagZ;

sensor_data_tmp.OrientY = -sensor_data.OrientY;

sensor_data = sensor_data_tmp;
sensor_data_orient = sensor_data;
imu_data_size = size(sensor_data.Timestamp,1);
gps_data_size = size(mobile_data.Position.Timestamp,1);
% has a lot of gps data then it is good
if (gps_data_size > (imu_data_size*sample_dt)/2)
    gps_valid = 1;
    sensor_data_imu_gps = synchronize(sensor_data, mobile_data.Position);
else
    gps_valid = 0;
    sensor_data_imu_gps = sensor_data_orient;
end

filter_times = seconds(sensor_data_imu_gps.Timestamp-sensor_data_imu_gps.Timestamp(1));
sensor_data_imu_gps = sensor_data_imu_gps(filter_times <= total_time,:);

end