% Dan Nguyen - z5206032
function [X, P] = kalman_filter(X, P, H, R, Z)
    S = R + H*P*H';
    K = P*H'*inv(S);
    P = P - K*H*P;
    X = X + K*Z;
end