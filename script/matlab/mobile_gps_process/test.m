

folder_path = '/home/shuoyang/Documents/vilo_dev/vilo_ws/bags/230525/';

file_name = '20230525_frick_park_arch_short.mat';

position = position_filter([folder_path, file_name]);

file_name = 'wightman_right_loop_trot.mat';

position = position_filter([folder_path, file_name], 350 ,0);
%%
position = movmean(position,150,1);
plot3(position(:,1),position(:,2),position(:,3))
axis equal
% % mobile_data = load(file_path);
% SampleRate = 100.0;
% [mobile_data,gps_valid,mobile_data_imu_gps] = process_mobile_data(file_path,1/SampleRate);
% 
% % sensor data are all in NED
% Accelerometer = [mobile_data.AccX mobile_data.AccY mobile_data.AccZ];
% Gyroscope = [mobile_data.GyroX mobile_data.GyroY mobile_data.GyroZ];
% Magnetometer = [mobile_data.MagX mobile_data.MagY mobile_data.MagZ];
% qTrue = quaternion([mobile_data.OrientZ mobile_data.OrientY mobile_data.OrientX], ...
%     "eulerd","ZYX","frame");
% 
% % get the exompass, use the first 4 as average 
% q = ecompass(Accelerometer,Magnetometer);
% Navg = 4;
% qfix = meanrot(q(1:Navg))./meanrot(qTrue(1:Navg));
% Orientation = qfix*qTrue; % Rotationally corrected phone data.
% 
% orientFilt = ahrsfilter(SampleRate=SampleRate);
% groundTruth = table(Orientation);
% sensorData = table(Accelerometer,Gyroscope,Magnetometer);
% 
% tc = tunerconfig("ahrsfilter",MaxIterations=30, ...
%     ObjectiveLimit=0.001,Display="none");
% tune(orientFilt,sensorData,groundTruth,tc);
% 
% reset(orientFilt);
% qEst = orientFilt(Accelerometer,Gyroscope,Magnetometer);
% 
% numSamples = numel(Orientation);
% t = (0:numSamples-1).'/SampleRate;
% 
% d = rad2deg(dist(qEst, Orientation));
% 
% figure
% plot(t,eulerd(qEst,"ZYX","frame"))
% legend yaw pitch roll
% title("ahrsfilter Euler Angles")
% ylabel("Degrees")
% xlabel("Time (s)")
% 
% %% has gps, do insfilter
% if (gps_valid == 1)
%     % a relative tiem list
%     filter_times = seconds(mobile_data_imu_gps.Timestamp-mobile_data_imu_gps.Timestamp(1));
%     % find the first nonNaN gps as refloc
%     temp = mobile_data_imu_gps.latitude;
%     temp(~isnan(temp)) = 1;
%     temp(isnan(temp)) = 0;
%     temp = find(temp);
%     first_non_NaN_index_of_X = temp(1);
%     refloc = [mobile_data_imu_gps.latitude(first_non_NaN_index_of_X)
%           mobile_data_imu_gps.longitude(first_non_NaN_index_of_X)
%           mobile_data_imu_gps.altitude(first_non_NaN_index_of_X)];
% 
%     disp(refloc)
%     % construct initState
%     initState = zeros(22,1);
%     [q0,q1,q2,q3] = parts(Orientation(1));
%     initState(1) = q0;
%     initState(2) = q1;
%     initState(3) = q2;
%     initState(4) = q3;
%     initState(17:19) = Magnetometer(1,:)';
%     initState(20:22) = zeros(3,1);
% 
% 
%     filt = insfilterAsync;
%     filt.ReferenceLocation = refloc;
%     filt.State = [initState(1:4);0;0;0;initState(5:10);0;0;0;initState(11:end)];
%     filt.StateCovariance = 0.1*eye(28);
% 
%     filt.AccelerationNoise = 0.01*ones(3,1);
%     filt.AngularVelocityNoise = 0.001*ones(3,1);
%     filt.PositionNoise = 1e-2*ones(3,1);
%     filt.VelocityNoise = 1e-1*ones(3,1);
%     filt.QuaternionNoise = 1e-3*ones(4,1);
%     filt.GeomagneticVectorNoise = 1e-3*ones(3,1);
%     filt.GyroscopeBiasNoise = 1e-3*ones(3,1);
%     Rmag  = 10;
%     Racc  = 500;
%     Rgyro = 1;
%     Rpos  = 1;
% 
% 
%     N = size(mobile_data_imu_gps.AccX,1);
%     p = zeros(N,3);
%     q = zeros(N,1,'quaternion');
%     for ii = 1:N               % Fuse IMU
%         predict(filt,1./SampleRate);
%         if ~isnan(mobile_data_imu_gps.AccX(ii))
%             accel = [mobile_data_imu_gps.AccX(ii) 
%                      mobile_data_imu_gps.AccY(ii) 
%                      mobile_data_imu_gps.AccZ(ii)]';
%             gyro =  [mobile_data_imu_gps.GyroX(ii) 
%                      mobile_data_imu_gps.GyroY(ii) 
%                      mobile_data_imu_gps.GyroZ(ii)]';
%             mag = [mobile_data_imu_gps.MagX(ii) 
%                      mobile_data_imu_gps.MagY(ii) 
%                      mobile_data_imu_gps.MagZ(ii)]';
%            
%             fuseaccel(filt,accel,[Racc;Racc;Racc]);
%             fusegyro(filt,gyro,Rgyro);
%                     
%             fusemag(filt,mag,Rmag);
%         end
%         if ~isnan(mobile_data_imu_gps.latitude(ii))    
%             lla = [mobile_data_imu_gps.latitude(ii) 
%                 mobile_data_imu_gps.longitude(ii) 
%                 mobile_data_imu_gps.altitude(ii)]';
%           fusegps(filt,lla,[Rpos;Rpos;Rpos]);
%         end
%        [p(ii,:),q(ii)] = pose(filt);           %Log estimated pose
%        
%     end
% end
% %%
% % from NED to ros frame 
% R = [ 1 0 0 ;
%       0 -1 0;
%       0 0 -1];
% euler = eulerd(Orientation(1),"ZYX","frame");
% init_yaw = -euler(1)/180*pi;
% R_yaw = [cos(init_yaw) sin(init_yaw) 0;
%         -sin(init_yaw) cos(init_yaw) 0;
%          0 0 1];
% 
% ros_p = p * R'*R_yaw';
% figure(2)
% plot3(ros_p(:,1),ros_p(:,2),ros_p(:,3))
% view(-90,90);
% axis equal
% 
% % imuFs = 100; % iphone output
% % refloc = [mobile_data.Position(1,:).latitude 
% %           mobile_data.Position(1,:).longitude
% %           mobile_data.Position(1,:).altitude]; % refloc is the first GPS data (should we make sure it is stable?)
% % 
% % initstate = zeros(22,1);
% % 
% % filter = insfilterMARG('IMUSampleRate', imuFs, 'ReferenceLocation', ...
% %         refloc, 'AccelerometerBiasNoise', 2e-4, ...
% %         'AccelerometerNoise', 2, 'GyroscopeBiasNoise', 1e-16, ...
% %         'GyroscopeNoise', 1e-5, 'MagnetometerBiasNoise', 1e-10, ...
% %         'GeomagneticVectorNoise', 1e-12, 'StateCovariance', ...
% %         1e-9*ones(22), 'State', initstate);