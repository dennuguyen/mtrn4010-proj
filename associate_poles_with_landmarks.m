% Dan Nguyen - z5206032
% Perform a data association to filter for poles that are associated with a
% landmark.
function [associated_poles_indexes] = associate_poles_with_landmarks(points, indexes, landmarks, threshold)
    associated_poles_indexes = zeros(1, length(indexes));
    s = size(points);
    length_poles = s(2);
    for i = 1:length(landmarks)
        for j = 1:length_poles
            if indexes(j) == 1 && norm(landmarks(:, i) - points(:, j)) < threshold
                associated_poles_indexes(j) = 1;
            end
        end
    end
end