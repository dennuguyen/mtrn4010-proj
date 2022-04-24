%
% true_observation is the landmark in the GCF.
% measured_observation is the measured landmark in the GCF.
function [Z, H, S] = innovate(state_vector, state_covariance, true_observation, measured_observation, observation_noise_covariance)
    %% Calculate innovation.
    observation_model = @(X, Xk) [sqrt((X(1) - Xk(1))^2 + ((X(2) - Xk(2)))^2)
                                  atan2(Xk(2) - X(2), Xk(1) - X(1)) - X(3)];
    polar_true_observation = observation_model(state_vector, true_observation);
    polar_measured_observation = observation_model(state_vector, measured_observation);
    Z = polar_true_observation - polar_measured_observation;

    %% Calculate Jacobian matrix of observation model.
    dx = state_vector(1) - measured_observation(1);
    dy = state_vector(2) - measured_observation(2);
    H = [dx / polar_measured_observation(1), dy / polar_measured_observation(1), 0;
         -dy / polar_measured_observation(1)^2, dx / polar_measured_observation(1), -1];

    %% Calculate innovation covariance.
    S = H * state_covariance * H' + observation_noise_covariance;
end