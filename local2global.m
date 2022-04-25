% Dan Nguyen - z5206032 - 25/04/2022
% Transform local to global coordinates.
function [global_points] = local2global(local_points, transform_vector)
    rotation = transform_vector(3);
    R = [cos(rotation) -sin(rotation); sin(rotation) cos(rotation)];
    x_y_rotated = R * local_points;
    global_points(1, :) = x_y_rotated(1, :) + transform_vector(1);
    global_points(2, :) = x_y_rotated(2, :) + transform_vector(2);
end