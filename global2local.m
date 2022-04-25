% Dan Nguyen - z5206032 - 25/04/2022
% Transform global to local coordinates.
function [local_points] = global2local(global_points, transform_vector)
    rotation = transform_vector(3);
    R = [cos(rotation) sin(rotation); -sin(rotation) cos(rotation)];
    x_y_rotated = R * global_points;
    local_points(1, :) = x_y_rotated(1, :) + transform_vector(1);
    local_points(2, :) = x_y_rotated(2, :) + transform_vector(2);
end