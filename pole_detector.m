function [poles] = pole_detector(points, pole_size, point_count_threshold, max_range)
    point_pole_index = zeros(1 , length(points));
    num_points_in_cluster = 0; % Count of points within a cluster.
    
    for i = 1:(length(points) - 1)
        distance = norm(points(:, i) - points(:, i + 1));
        
        % Cluster points that are within a max range.
        if distance < max_range
            num_points_in_cluster = num_points_in_cluster + 1;
        else
            % Cluster diameter is distance between first and last point.
            diameter = norm(points(:, i) - points(:, i - num_points_in_cluster));
            if pole_size(1) <= diameter && diameter <= pole_size(2) && num_points_in_cluster <= point_count_threshold
                point_pole_index(i) = 1;
            end
            num_points_in_cluster = 0;
        end
    end
    
    % Does a good job at removing false positives on the edges of the scan.
    point_pole_index(1:30) = 0;
    point_pole_index(end-30:end) = 0;
    
    % Filter the points for poles.
    poles = points(:, point_pole_index ~= 0);
end