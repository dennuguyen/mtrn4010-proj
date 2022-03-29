% Dan Nguyen - z5206032
% Computes the trilateration of two given points.
function [pose] = trilaterate(marks, initial_value)
    marks = double(marks);
    initial_value = double(initial_value);
    ranges = sqrt(marks(1, :).^2 + marks(2, :).^2);
    fn = @(i) [sqrt((i(1) - marks(1, 1))^2 + (i(2) - marks(2, 1))^2) - ranges(1);
               sqrt((i(1) - marks(1, 2))^2 + (i(2) - marks(2, 2))^2) - ranges(2)];
    pose = fsolve(fn, initial_value);
end