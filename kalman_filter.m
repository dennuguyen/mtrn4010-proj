function [state_vector, state_covariance] = kalman_filter(state_vector, state_covariance, innovation, innovation_jacobian, innovation_covariance)
    K = state_covariance * innovation_jacobian' / innovation_covariance;
    state_vector = state_vector + K * innovation;
    state_covariance = state_covariance - K * innovation_jacobian * state_covariance;
end