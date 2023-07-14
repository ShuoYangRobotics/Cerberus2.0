function [geo_pos] = ros_pos_to_geo_pos(ros_pos, yaw_offset, refloc)
%ros_pos_to_geo_pos ros_pos is Nx3, convert to geo pos
%   Detailed explanation goes here

%%first conver to NED
% from  ros frame to NED
R = [ 1 0 0 ;
      0 -1 0;
      0 0 -1];

R_yaw = [cos(yaw_offset/180*pi) sin(yaw_offset/180*pi) 0;
        -sin(yaw_offset/180*pi) cos(yaw_offset/180*pi) 0;
         0 0 1];

p = ros_pos * R * R_yaw';

lat0 = refloc(1);
lon0 = refloc(2);
h0 = refloc(3);

wgs84 = wgs84Ellipsoid;
[lat,lon,h] = ned2geodetic(p(:,1),p(:,2),0*p(:,3),lat0,lon0,h0,wgs84);

geo_pos = [lat';lon';h']';

end