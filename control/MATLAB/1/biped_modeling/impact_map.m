function Delta = impact_map(x, data)
    % 4.2) Extract relevant data

    % Extract q and qdot from state vector x
    n = length(x) / 2; % Assuming the state vector size is twice the number of joints
    q = x(1:n);
    qdot = x(n+1:end);
    
    % Extract necessary variables from data structure
    Dbar = data.Dbar(q);
    E = data.E(q);
    
    % 4.3) Compute impact map
    
    % Delta 1: Effect of impulsive forces
    Deltaqdot = ...
        [ eye(n), zeros(n, 4) ] * ...
        inv([ Dbar -E' ; E zeros(2) ]) * ...
        [ Dbar * [ eye(n); zeros(2, n) ] ; zeros(2, n) ];
    
    % Apply Delta1 to (q, qdot)
    q0 = q;
    q0dot = Deltaqdot * qdot;
    
    % Delta 2: Relabelling
    R = [1 1 1 1 0; 
         0 0 0 -1 1; 
         0 0 -1 0 0; 
         0 -1 0 0 0; 
         0 0 -1 0 1]; 
    d = [-3; 2; 2; 2; 1] * pi;
    
    % Delta: Impact map (apply Delta2 to Delta1)
    Delta = [ R * q0 + d ; R * q0dot ];
end
