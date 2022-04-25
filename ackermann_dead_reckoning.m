% Dan Nguyen - z5206032 - 25/04/2022
% Calculate the next pose from previous pose based on velocities and change in time for the Ackermann model.
function next_pose = ackermann_dead_reckoning(pose, linear_velocity, angular_velocity, change_in_time)
    velocity = [linear_velocity * cos(pose(3));
                linear_velocity * sin(pose(3));
                angular_velocity - pose(4);
                0];
    next_pose = pose + velocity * change_in_time; % s_f - s_i = v * dt
end