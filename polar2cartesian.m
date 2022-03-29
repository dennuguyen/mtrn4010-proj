% Dan Nguyen - z5206032
% Transform polar to cartesian coordinates.
function [cartesian_points] = polar2cartesian(lidar_ranges, lidar_angles)
    x = lidar_ranges .* cos(lidar_angles);
    y = lidar_ranges .* sin(lidar_angles);
    cartesian_points = [x; y];
end