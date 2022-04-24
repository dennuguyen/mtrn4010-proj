function [Z, H, S] = innovate(X, true_observation, measured_observation, R)
    dx = X(1) - measured_observation(1);
    dy = X(2) - measured_observation(2);
    range = sqrt(dx^2 + dy^2);

    %% Calculate innovation.
    polar_true_observation = global2local(true_observation, X);
    polar_measured_observation = [range, atan2(dy, dx) - X(3)];
    Z = U - u;

    %% Calculate Jacobian matrix of observation model.
    H = [dx / range, dy / range, 0];

    %% Calculate innovation covariance.
    S = H * P * H' + R;
end