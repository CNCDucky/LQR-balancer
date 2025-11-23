function s = commat(M)
    % Converts a MATLAB matrix or vector
    % into a comma-separated string
    
    [n, m] = size(M);
    parts = strings(n, m);
    
    for i = 1:n
        for j = 1:m
            parts(i,j) = sprintf('%g', M(i,j));
        end
    end
    
    % Join each row into one string
    rows = strings(n,1);
    for i = 1:n
        rows(i) = strjoin(parts(i,:), ', ');
    end
    
    % Join rows with newlines
    s = char(strjoin(rows, newline));
end