% Dan Nguyen - z5206032 - 25/04/2022
% Gyoroscope bias cost function based on landmark observations.
function [cost] = landmark_cost(guess, pose, landmark_details, change_in_time)
    cost = 0;
    observed_landmarks = [];
    truth_landmarks = [];
    for i = 1:length(landmark_details)
        if cell2mat(landmark_details(i)) ~= 0
            unpacked = cell2mat(landmark_details(2, i));
            if isempty(unpacked) == false
                observed_landmarks = [observed_landmarks unpacked(:, 1)];
                truth_landmarks = [truth_landmarks unpacked(:, 2)];
            end
        end
    end
    if isempty(observed_landmarks) || isempty(truth_landmarks)
        return
    end
    angles = double(atan2(truth_landmarks(2, :) - pose(2), truth_landmarks(1, :) - pose(1)) - atan2(observed_landmarks(2, :) - pose(2), observed_landmarks(1, :) - pose(1))) .* change_in_time;
    guesses = double(zeros(1, length(angles)) + guess * change_in_time);
    cost = sum((guesses - angles).^2);
end