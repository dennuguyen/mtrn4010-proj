% Dan Nguyen - z5206032
% Convert LiDAR ranges to polar coordinates.
function [magnitudes, angles] = ranges2polar(ranges, unit_conversion, field_of_view, angular_resolution, thresholds)
    magnitudes = double(ranges) * unit_conversion;
    angles = deg2rad(field_of_view(1):angular_resolution:field_of_view(2));
    valid_ranges = find((magnitudes > thresholds(1)) & (magnitudes < thresholds(2)));
    magnitudes = magnitudes(valid_ranges)';
    angles = angles(valid_ranges);
end