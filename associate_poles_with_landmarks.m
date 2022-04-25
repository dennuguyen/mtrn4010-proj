% Dan Nguyen - z5206032 - 25/04/2022
% Perform a data association to filter for poles that are associated with a
% landmark.
function [associated_poles_indexes, index_map, landmark_details] = associate_poles_with_landmarks(poles, indexes, landmarks, landmark_details, threshold)
    associated_poles_indexes = zeros(1, length(indexes));
    index_map = containers.Map("KeyType", "uint32", "ValueType", "uint32");
    s = size(poles);
    length_poles = s(2);
    for i = 1:length(landmarks)
        if cell2mat(landmark_details(1, i)) == 1
            continue
        end
        for j = 1:length_poles
            if indexes(j) == 1 && norm(landmarks(:, i) - poles(:, j)) < threshold
                associated_poles_indexes(j) = 1;
                index_map(j) = i;
                landmark_details(1, i) = {1};
                landmark_details(2, i) = {double([landmarks(:, i) poles(:, j)])};
                continue
            end
        end
    end
end