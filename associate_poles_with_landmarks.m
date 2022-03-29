% Dan Nguyen - z5206032
% Perform a data association to filter for poles that are associated with a
% landmark.
function [associated_poles] = associate_poles_with_landmarks(poles, landmarks, threshold)
    associated_poles = [];
    s = size(poles);
    length_poles = s(2);
    for i = 1:length(landmarks)
        for j = 1:length_poles
            if norm(landmarks(:, i) - poles(:, j)) < threshold
                associated_poles = [associated_poles poles(:, j)];
            end
        end
    end
end