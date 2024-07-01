function data = constraint_dynamics_check(data, display_VM_graph)

    fprintf('\nDerivation of constrained dynamics...\n')
    syms theta real

    sigma = data.sigma_fun;
    sigmaprime = data.sigmaprime_fun;
    sigmapprime = data.sigmapprime_fun;
    D = data.D;
    G = data.G;
    C = data.C; 
    B_perp = data.B_perp;

    denominatorfun = @(theta) B_perp*D(sigma(theta))*sigmaprime(theta); 
    psi1fun = @(theta) -B_perp*G(sigma(theta))/denominatorfun(theta);
    psi2fun = @(theta) -B_perp * (D(sigma(theta))*sigmapprime(theta)+ C([sigma(theta);sigmaprime(theta)]) * sigmaprime(theta)) /denominatorfun(theta);
    data.Psi1 = psi1fun;
    data.Psi2 = psi2fun;
    
    % Calculating M and V numerically using the ODE function: 
    fprintf('\n...finding virtual mass and virtual potential...\n')
    ops2=odeset('RelTol',1e-4,'AbsTol',1e-4);
    [Theta,X]=ode45(@mass_potential,linspace(0,1,1000)',[1;0],ops2,data);
    M=spline(Theta,X(:,1));
    V=spline(Theta,X(:,2));
    data.M = M;
    data.V = V; 
    
    % Evaluate M and V at theta = 1 and find Vmax
    M_minus = ppval(M, 1);
    V_minus = ppval(V, 1);
    V_max = max(ppval(V, linspace(0, 1, 1000)));
    data.M_minus = M_minus;
    data.V_minus = V_minus; 
    data.V_max = V_max;
    
    % Compute delta
    sigma_prime_0 = sigmaprime(0);
    sigma_prime_1 = sigmaprime(1);
    I = data.I; % Impact matrix
    delta = (sigma_prime_0.' * I * sigma_prime_1) / (sigma_prime_0.' * sigma_prime_0);
    data.delta = delta;

    if display_VM_graph
        % Stability conditions
        condition1 = (delta^2 / M_minus) < 1;
        condition2 = ((V_minus * delta^2) / (M_minus - delta^2)) + V_max < 0;
        
        % Output the results
        fprintf('Stability Condition 1 (0 < delta^2 / M_minus < 1): %d\n', condition1);
        fprintf('Stability Condition 2 ((V_minus * delta^2) / (M_minus - delta^2) + V_max < 0): %d\n', condition2);
        
        if condition1 && condition2
            disp('SUCCESS! - The VHC induces a stable hybrid limit cycle.');
        else
            disp('FAILED! - The VHC does not induce a stable hybrid limit cycle. Check the implementation or parameter values.');
        end
        
        % Visualization for M and V vs theta
        figure
        plot(Theta,X(:,1));
        hold on
        plot(Theta,X(:,2));
        legend('M','V')
        drawnow;
        hold off;
    end 
end 


function xdot = mass_potential(theta, x, data)
    M = x(1);
    V = x(2);
    Mdot = -2 * M * data.Psi2(theta);
    Vdot = -data.Psi1(theta) * M;
    xdot = [Mdot; Vdot];
end