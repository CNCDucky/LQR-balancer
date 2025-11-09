function x_filtered = EKFparamEstimator(y, x0, P0, f, Q, h, R)
    % Estimates states using a nonlinear motion
    % model fx and its linearization Fx

    %   y           [m x 1] Current measurement
    %   x0          [n x 1] Prior mean for time 0
    %   P0          [n x n] Prior covariance
    %   f                   Motion model function handle
    %                       [fx,Fx]=f(x) 
    %                       Takes as input x (state) 
    %                       Returns fx and Fx, motion model and Jacobian evaluated at x 
    %   Q           [n x n] Process noise covariance
    %   h                   Measurement model function handle
    %                       [hx,Hx]=h(x,T) 
    %                       Takes as input x (state), 
    %                       Returns hx and Hx, measurement model and Jacobian evaluated at x
    %   R           [m x m] Measurement noise covariance
    %
    % Output:
    %   x_filtered  [n x 1]

        persistent P x

        if isempty(P)
            x = x0;
            P = P0;
        end

        % Assume gaussian prior!
        % Take in prior mean x.
        % linear approximation of f linearixed around x,
        % this is to create a linear transformation
        % with approximate prediction mean and covariance

        [fx,Fx] = f(x);
        x = fx;                 % Predicted state
        P = Fx*P*Fx.' + Q;      % Filter error covariance
        
        [hx,Hx] = h(x);
        y_pred = hx;            % Predicted measurement

        S = Hx*P*Hx.' + R;      % Innovation covariance
        K = P*Hx.'/ S;          % Kalman gain
        v = y - y_pred;         % Innovation/corrected measurement

        x_filtered = x + K*v;   % Predicted state corrected by kalman gain
        % P = P - K*S*K.';      % Cov is reduced by measurement
        I = eye(size(P));
        P = (I - K*Hx)*P*(I - K*Hx).' + K*R*K.';  % Joseph form (stable)

        x = x_filtered;         % New prior for next iteration

end