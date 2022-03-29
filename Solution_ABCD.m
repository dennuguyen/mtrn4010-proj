% Dan Nguyen - z5206032
% Main solution which solves parts A, B, C, D.

%% Load data.
load("data015a.mat");

%% Set up dynamic plot.
figure(1)
clf
hold on
pose_plot = plot(-7, -7, 'b*');
heading_plot = plot(-7, -7, 'm*');
point_cloud_plot = plot(-7, -7, 'r.');
poles_plot = plot(-7, -7, 'ko');
localised_pose_plot = plot(-7, -7, 'g+');
axis([-5, 20, -5, 20])

%% Initial data.
pose = data.pose0;
heading = 0;
point_clouds = [];
poles = [];
localised_pose = data.pose0(1:2);
prev_time = 0.0001 * double(data.table(1, 1));
linear_velocity = 0;
angular_velocity = 0;

for i = 1:data.n
    %% Get data for this iteration.
    next_time = 0.0001 * double(data.table(1, i));
    index = data.table(2, i);
    sensor_id = data.table(3, i);
    
    %% Calculate time.
    change_in_time = next_time - prev_time;
    prev_time = next_time;
    
    %% Calculate pose.
    pose = ackermann_dead_reckoning(pose, linear_velocity, angular_velocity, change_in_time);
    heading = 2 * [cos(pose(3)); sin(pose(3))] + [pose(1); pose(2)];
    
    %% Read sensor data.
    if sensor_id == 1 % Update pose.
        ranges = data.scans(:, index);
        [ranges, angles] = ranges2polar(ranges, 0.01, [-80, 80], 0.5, [1, 20]);
        local_point_cloud = polar2cartesian(ranges, angles);
        offset = [0.4 * cos(pose(3)); 0.4 * sin(pose(3)); 0];
        point_cloud = local2global(local_point_cloud, pose + offset);
        point_clouds = [point_clouds point_cloud];
        
        %% Detect sweeps of poles.
        potential_poles = pole_detector(point_cloud, [0.05 0.2], 6, 0.8);

        %% Cleans up the list of poles by data associating the sweeps of poles with given landmarks.
        associated_poles = associate_poles_with_landmarks(potential_poles, data.Landmarks, 0.2);
        poles = [poles associated_poles];
        
        %% Localise platform using trilateration.
        if isempty(poles) == false
            two_random_poles = poles(randi(numel(poles), 2));
            two_random_poles = global2local(two_random_poles, pose);
            localised_local_pose = trilaterate(two_random_poles, pose(1:2));
            localised_global_pose = local2global(localised_local_pose, pose);
            localised_pose = [localised_pose localised_global_pose];
        end

    elseif sensor_id == 2 % Update velocities.
        linear_velocity = data.vw(1, index);
        angular_velocity = data.vw(2, index);
    else
        disp("Not valid sensor_id")
    end
    
    %% Update dynamic plot.
    plot_and_check_empty(pose_plot, pose);
    plot_and_check_empty(heading_plot, heading);
    plot_and_check_empty(point_cloud_plot, point_clouds);
    plot_and_check_empty(poles_plot, poles);
    plot_and_check_empty(localised_pose_plot, localised_pose);
    pause(0.001)
end

function plot_and_check_empty(handle, vec)
    if isempty(vec) == false
        set(handle, 'xdata', vec(1, :), 'ydata', vec(2, :));
    end
end

%% Validation.
% plot(poses(1, :), poses(2, :), 'b.')
% hold on
% plot(data.verify.poseL(1, :), data.verify.poseL(2, :), 'r');