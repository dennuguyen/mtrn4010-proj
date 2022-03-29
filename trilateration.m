function [pose] = trilateration(r1, r2, x1, x2, y1, y2)
    fn = @(u) [sqrt((u(1) - x1)^2 + (u(2) - y1)^2) - r1;
               sqrt((u(1) - x2)^2 + (u(2) - y2)^2) - r2];
    pose = fsolve(fn, [0 0]);
end