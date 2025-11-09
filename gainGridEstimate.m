function [A_blend, B_blend, K_blend, w] = gainGridEstimate(y, H, R, x_prev, P_prev, A_grid, Q, B_grid, u_prev, K_grid, w_old)
    % Simulates the predicted states given the different grid models
    % Calculates the likelihood of the innovations for the different
    % models.
    % The innovation is the difference between what is measured, and the
    % predicted measurement. The difference between them comes from 
    % modelling error, noise and disturbance. 
    % Given the likelihood for the models, calculate the weight of each
    % gain in the grid by comparing each likelihood to the sum of all
    % likelihoods.

    n_models = size(A_grid, 1);
    m_models = size(A_grid, 2);
    logL = zeros(n_models, m_models);

    % low pass filter
    alpha = 0.99;

    for i = 1:n_models
        for j = 1:m_models

            % Prediction
            A = A_grid{i,j};
            B = B_grid{i,j};
            x_pred = A*x_prev + B*u_prev;
            P = A*P_prev*A.' + Q;
            
            % Innovation
            S = H*P*H.' + R;
            S = S + 1e-6*eye(size(S)); % Regularize
            v = y - H*x_pred;

            % Calulate log likelihood
            % Mahalanobis distance => higher likelihood => higher weight
            % logL = -1/2�(v.'/�S*�v + log(det(S)�)) + length(v)*log(2*pi))
            % Simplified for computational reasons :)
            logL(i,j) = -0.5*(v.'*(S\v));   % -0.5 * Mahalanobis distance squared, probably close enough

        end
    end

    % Normalize LogL to get weights
    L = exp(logL - max(logL(:)));   % stabilizes
    w = L./sum(L(:));

    w = alpha*w_old + (1-alpha)*w;
    
    A_blend = zeros(4,4);
    B_blend = zeros(4,1);
    K_blend = zeros(1,4);

    for i = 1:n_models
        for j = 1:m_models
            A_blend = A_blend + A_grid{i,j}*w(i,j);
            B_blend = B_blend + B_grid{i,j}*w(i,j);
            K_blend = K_blend + K_grid{i,j}*w(i,j);
        end
    end
end


