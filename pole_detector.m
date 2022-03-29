% Dan Nguyen - z5206032
% Detects the sweeps of poles.
function [poles] = pole_detector(points, pole_size, point_count_threshold, max_range)
    num_points_in_cluster = 0; % Count of points within a cluster.
    poles = [];
    
    for i = 1:(length(points) - 1)
        % Get distance between ith and (i + 1)th point.
        distance = norm(points(:, i) - points(:, i + 1));
        
        % Cluster points that are within a max range.
        if distance < max_range
            num_points_in_cluster = num_points_in_cluster + 1;
        else
            % Cluster diameter is distance between first and last point.
            diameter = norm(points(:, i) - points(:, i - num_points_in_cluster));
            if pole_size(1) <= diameter && diameter <= pole_size(2) && num_points_in_cluster <= point_count_threshold
                poles = [poles points(:, i)];
            end
            num_points_in_cluster = 0;
        end
    end
end