% Dan Nguyen - z5206032
% Perform a data association to filter for poles that are associated with a
% landmark.
function [associated_poles_indexes, index_map, seen_landmarks] = associate_poles_with_landmarks(poles, indexes, landmarks, seen_landmarks, threshold)
    associated_poles_indexes = zeros(1, length(indexes));
    index_map = containers.Map("KeyType", "uint32", "ValueType", "uint32");
    s = size(poles);
    length_poles = s(2);
    for i = 1:length(landmarks)
        if seen_landmarks(i) == 1
            continue
        end
        for j = 1:length_poles
            if indexes(j) == 1 && norm(landmarks(:, i) - poles(:, j)) < threshold
                associated_poles_indexes(j) = 1;
                index_map(j) = i;
                seen_landmarks(i) = 1;
                continue
            end
        end
    end
end