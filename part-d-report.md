# Part D Report

Part D requires you to localise the platform's pose (at least position) with trilateration and/or triangulation.

I used 2 trilaterion equations and simultaneously solved them for the vehicle's pose in the local coordinate frame. My poles were in the global coordinate frame so I had x-y values but I had no information about the range variable required for trilateration. So, I picked two random poles and transformed them into the local coordinate frame of the vehicle then used Pythogaras' theorem with origin (0, 0) to calculate the range.

The code for my part D:
```MATLAB
% Gets 2 random poles from a list of poles that have been data associated with landmarks.
two_random_poles = poles(randi(numel(poles), 2));

% Convert the 2 random poles into local frame so we can calculate the distance from the origin to the pole.
two_random_poles = global2local(two_random_poles, pose);

% Trilaterate the vehicle's position in the local coordinate frame. Pass in pose(1:2) for initial values for fsolve.
localised_local_pose = trilaterate(two_random_poles, pose(1:2));

% Convert the output of trilaterate into the global coordinate frame so we can plot it.
localised_global_pose = local2global(localised_local_pose, pose);

function [pose] = trilaterate(marks, initial_value)
    % Convert everything to double because fsolve will complain.
    marks = double(marks);
    initial_value = double(initial_value);

    % Calculate the ranges. Do not need distance formula since origin is the vehicle's pose.
    ranges = sqrt(marks(1, :).^2 + marks(2, :).^2);

    % Solve the trilateration equations with fsolve.
    fn = @(i) [sqrt((i(1) - marks(1, 1))^2 + (i(2) - marks(2, 1))^2) - ranges(1);
               sqrt((i(1) - marks(1, 2))^2 + (i(2) - marks(2, 2))^2) - ranges(2)];
    pose = fsolve(fn, initial_value);
end
```