function x_filtered = kalmanFilter(y, H, R, x0, P0, A, Q, B, u)

    %   y           [m x 1] Current measurement
    %   u           [n x r] previous iteration input
    %   x0          [n x 1] Initial prior mean
    %   P0          [n x n] Initial prior covariance
    %   A           [n x n] State transition matrix
    %   Q           [n x n] Process noise covariance
    %   H           [m x n] Measurement model matrix
    %   R           [m x m] Measurement noise covariance

    % Output:
    %   x           [n x 1] Estimated state vector
    %   P           [n x n] Filter error convariance
    
    % N = size(y,2);
    % n = length(x0);

    persistent P x

    if isempty(P)
        x = x0;
        P = P0;
    end
    
    % Prediction
    x_pred = A*x + B*u;     % Predicted state
    P = A*P*A.' + Q;        % Filter error covariance
    
    % Measurement update
    S = H*P*H.' + R;        % Innovation covariance
    K = P*H.'*inv(S);       % Kalman gain
    v = y - H*x_pred;       % Innovation/corrected measurement
    
    x_filtered = x_pred + K*v;       % Predicted state corrected by kalman gain
    P = P - K*S*K.';        % Cov is reduced by measurement

    x = x_filtered; % New prior for next iteration
end