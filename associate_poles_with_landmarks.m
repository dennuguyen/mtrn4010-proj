function [associated_poles] = associate_poles_with_landmarks(poles, landmarks, threshold)
    associated_poles = {};
    for i = 1:length(landmarks)
        for j = 1:length(poles)
            if norm(landmarks(:, i) - poles(:, j)) < threshold
                associated_poles = cat(2, associated_poles, [landmarks(:, i) poles(:, j)]);
                break
            end
        end
    end
end