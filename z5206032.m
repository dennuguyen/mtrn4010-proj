load("data015a.mat");

%% Initial data.
pose = data.pose0;
heading = 2 * [cos(pose(3)); sin(pose(3))] + [pose(1); pose(2)];
point_clouds = [0; 0];
poles_and_landmarks = {};
poles = [0; 0];
prev_time = 0.0001 * double(data.table(1, 1));
linear_velocity = 0;
angular_velocity = 0;

%% Set up dynamic plot.
figure(1)
clf
hold on
pose_plot = plot(pose(1), pose(2), 'b*');
heading_plot = plot(heading(1), heading(2), 'g*');
point_cloud_plot = plot(point_clouds(1), point_clouds(2), 'r.');
poles_plot = plot(poles(1), poles(2), 'ko');
landmarks_plot = plot(poles(1), poles(2), 'm+');
axis([-5, 20, -5, 20])

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
    
    %% Triangulation and trilateration.
    
    %% Read sensor data.
    if sensor_id == 1 % Update pose.
        ranges = data.scans(:, index);
        [ranges, angles] = ranges2polar(ranges, 0.01, [-80, 80], 0.5, [1, 20]);
        local_point_cloud = polar2cartesian(ranges, angles);
        offset = [0.4 * cos(pose(3)); 0.4 * sin(pose(3)); 0];
        point_cloud = local2global(local_point_cloud, pose + offset);
        point_clouds = [point_clouds point_cloud];
        
        %% Detect sweeps of poles.
        poles = [poles pole_detector(point_cloud, [0.05 0.2], 1, 0.8)];

        %% Data associate the sweeps of poles with given landmarks.
%         poles_and_landmarks = cat(2, poles_and_landmarks, associate_poles_with_landmarks(poles, data.Landmarks, 1));

    elseif sensor_id == 2 % Update velocities.
        linear_velocity = data.vw(1, index);
        angular_velocity = data.vw(2, index);
    else
        disp("Not valid sensor_id")
    end
    
    %% Update dynamic plot.
    set(pose_plot, 'xdata', pose(1), 'ydata', pose(2), 'color', 'b', 'marker', '*');
    set(heading_plot, 'xdata', heading(1), 'ydata', heading(2), 'color', 'g', 'marker', '*');
    set(point_cloud_plot, 'xdata', point_clouds(1, :), 'ydata', point_clouds(2, :), 'color', 'r', 'marker', '.');
    set(poles_plot, 'xdata', poles(1, :), 'ydata', poles(2, :), 'color', 'k', 'marker', 'o');
%     for i = 2:length(poles_and_landmarks)
%         set(landmarks_plot, 'xdata', poles_and_landmarks{i}(1, 1), 'ydata', poles_and_landmarks{i}(2, 1), 'color', 'm', 'marker', '+');
%     end
    pause(0.001)
end

%% Validation.
plot(poses(1, :), poses(2, :), 'b.')
hold on
plot(data.verify.poseL(1, :), data.verify.poseL(2, :), 'r');