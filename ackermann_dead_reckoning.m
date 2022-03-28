%% Calculate the next pose from previous pose based on velocities and change in time.
function next_pose = ackermann_dead_reckoning(pose, linear_velocity, angular_velocity, change_in_time)
    velocity = [linear_velocity * cos(pose(3));
                linear_velocity * sin(pose(3));
                angular_velocity];
    next_pose = pose + velocity * change_in_time; % s_f - s_i = v * dt
end