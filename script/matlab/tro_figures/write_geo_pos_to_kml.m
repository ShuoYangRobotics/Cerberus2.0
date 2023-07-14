function  write_geo_pos_to_kml(geo_pos, filename, color_code)

kmlwriteline(filename, geo_pos(:,1), geo_pos(:,2), 'Color', color_code, ...
       'LineWidth', 3);

end