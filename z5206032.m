% Dan Nguyen - z5206032
% Main solution which solves parts A, B, C, D.

%% Load data.
load("data019b.mat");

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
initial_value = data.pose0(1:2);
localised_pose = data.pose0(1:2);
prev_time = 0.0001 * double(data.table(1, 1));
linear_velocity = 0;
angular_velocity = 0;

%% Initial data for EKF.

sd_speed = 0.05;
sd_gyro = deg2rad(1);
sd_range = 0.1;
sd_bearing = deg2rad(2);

state_covariance = [0 0 0; 0 0 0; 0 0 0];
input_noise_covariance = [sd_speed^2 0; 0 sd_gyro^2];
observation_noise_covariance = [sd_range^2 0; 0 sd_bearing^2];

J = eye(3);
Ju = eye(3);

observation_model = @(X, Xk) [sqrt((X(1) - Xk(1))^2 + ((X(2) - Xk(2)))^2),  atan2(Xk(2) - X(2), Xk(1), X(1)) - X(3)];

for i = 1:data.n
    %% Get data for this iteration.
    next_time = 0.0001 * double(data.table(1, i));
    index = data.table(2, i);
    sensor_id = data.table(3, i);
    
    %% Calculate time.
    change_in_time = next_time - prev_time;
    prev_time = next_time;
    
    %% Read sensor data.
    if sensor_id == 1 % Update pose.
        ranges = data.scans(:, index);
        [ranges, angles] = ranges2polar(ranges, 0.01, [-80, 80], 0.5, [1, 20]);
        local_point_cloud = polar2cartesian(ranges, angles);
        offset = [0.4 * cos(pose(3)); 0.4 * sin(pose(3)); 0];
        point_cloud = local2global(local_point_cloud, pose + offset);
        point_clouds = [point_clouds point_cloud];

        %% Detect sweeps of poles.
        potential_poles_indexes = pole_detector(point_cloud, [0.05 0.2], 6, 0.8);

        %% Cleans up the list of poles by data associating the sweeps of poles with given landmarks.
        associated_poles_indexes = associate_poles_with_landmarks(point_cloud, potential_poles_indexes, data.Landmarks, 0.2);
        poles = [poles point_cloud(:, associated_poles_indexes == 1)];
        
        %% Get the innovation for the EKF.
        one_random_pole_index = randi(numel(associated_poles_indexes), 1);
        true_observation = data.Landmarks(:, one_random_pole_index);
        predicted_observation = predict_observation(point_cloud(:, one_random_pole_index));
        innovate(pose, true_observation, measured_observation, observation_noise_covariance);

        %% Localise platform using trilateration.
%         if isempty(poles) == false
%             two_random_poles_indexes = randi(numel(associated_poles_indexes), 2);
%             two_random_ranges = ranges(two_random_poles_indexes);
%             two_random_global_poles = point_cloud(:, two_random_poles_indexes);
%             localised_global_pose = trilaterate(two_random_global_poles, two_random_ranges, initial_value);
%             initial_value = localised_global_pose;
%             localised_pose = [localised_pose localised_global_pose];
%         end
        
        
        %% Perform EKF for pose.
        

    elseif sensor_id == 2 % Update velocities.
        linear_velocity = data.vw(1, index);
        angular_velocity = data.vw(2, index);
        
        %% Predict next state.
        [pose, state_covariance] = predict_next_state(pose, state_covariance, input_noise_covariance, change_in_time, linear_velocity, pose(3));
        heading = 2 * [cos(pose(3)); sin(pose(3))] + [pose(1); pose(2)];
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
