function [heading] = triangulation(a1, a2, x1, x2, y1, y2, x, y)
    fn = @(u) [atan2(y1 - y, x1 - x) - a1;
               atan2(y2 - y, x2 - x) - a2];
    heading = fsolve(fn, [0 0]);
end