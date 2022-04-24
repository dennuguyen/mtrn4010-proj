function [state_vector, state_covariance] = kalman_filter(state_vector, state_covariance, innovation, innovation_jacobian, innovation_covariance)
    kalman_gain = state_covariance * innovation_jacobian' / innovation_covariance;
    state_vector = state_vector + kalman_gain * innovation;
    state_covariance = state_covariance - kalman_gain * innovation_jacobian * state_covariance;
end