function [K, P, k] = getdLQR(Ad, Bd, P_prev, Q, R, tol, maxiter)
    %GETDLQR solves the ricatti equation and calcualtes stabilizing gain K
    % for a discrete time system. Since matlab's dlqr cannot use previous
    % Ricatti solutions P to calculate the gain, this function uses an
    % iterative approach from the definition of the discrete time Ricatti
    % equation initialized with the previous solution P. This makes it
    % converge much faster, which means it could be used for implementation
    % on a microcontroller

    % Minimized J = inf sum( x'Qx + u'Ru ), no cross coupling N

    A = Ad;
    B = Bd;

    k = 0;
    % Iterate Ricatti
    while k < maxiter

        P = A'*P_prev*A - (A'*P_prev*B)*((R + B'*P_prev*B)\(B'*P_prev*A)) + Q;
        k = k+1;

        % Check norm
        if norm(P - P_prev, 'fro')/norm(P, 'fro') <= tol
            break
        end
        
        P_prev = P;
    end
    
    % Calulate gain
    K = (R + B'*P*B)\(B'*P*A);

end