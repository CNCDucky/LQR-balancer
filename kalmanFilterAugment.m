function [z, Pz] = kalmanFilterAugment(y, Hz, R, z_prev, Pz_prev, Az, Qz)

    % Input:
    %   y           [m x 1] Current measurement
    %   u           [n x r] previous iteration input
    %   z           [x; u] Prior mean
    %   Pz          [Px, 0
    %                0, Pu] Prior covariance
    %   Az          [A, B
    %                0, 1] Augmented state transition matrix
    %   Qz          [Qx, 0
    %                0, Qu] Process noise covariance
    %   Hz          [H, 0] Measurement model matrix
    %   R           Measurement noise covariance

    % Output:
    %   z           [n x 1] Estimated state vector
    
    % Prediction
    z_pred = Az*z_prev;   % Predicted state
    Pz = Az*Pz_prev*Az.' + Qz;           % Filter error covariance
    
    % Measurement update
    S = Hz*Pz*Hz.' + R;         % Innovation covariance
    K = Pz*Hz.' / S;            % Kalman gain
    v = y - Hz*z_pred;          % Innovation/corrected measurement
    
    % Update filter estimate
    z = z_pred + K*v;           % Predicted state corrected by kalman gain
    Pz = Pz - K*S*K.';          % Cov is reduced by measurement

end