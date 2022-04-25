% Dan Nguyen - z5206032 - 25/04/2022
% Does the prediction step in the EKF.
function [X, P] = predict_next_state(X, P, Pu, Qf, dt, v)
    %% Calculate Jacobian matrices for transformation from current to next state.
    J = [1 0 -dt * v * sin(X(3)) 0; 0 1 dt * v * cos(X(3)) 0; 0 0 1 dt; 0 0 0 1];
    Ju = [dt * cos(X(3)) 0; dt * sin(X(3)) 0; 0 dt; 0 0];
    
    %% Calculate expected value of the state in next time step.
    X = J * X;
    
    %% Calculate the covariance matrix of the state in next time step.
    Q = J * P * J';
    Qu = Ju * Pu * Ju';
    P = Q + Qu + Qf;
end