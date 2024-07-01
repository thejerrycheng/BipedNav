function data = hybrid_limit_cycle_check(data, visualization)
    
    fprintf('\n...finding the constraint dynamics Psi1 and Psi2 ...\n')
    % Obtaining the requried functions from data structure: 
    sigma = data.sigma_fun;
    sigmaprime = data.sigmaprime_fun;
    sigmapprime = data.sigmapprime_fun;
    B_perp = data.B_perp;

    % Define theta range for plotting
    theta_range = linspace(0, 1, 100);
    
    % Evaluate sigma and its derivatives
    sigma_values = arrayfun(@(th) sigma(th), theta_range, 'UniformOutput', false);
    sigma_prime_values = arrayfun(@(th) sigmaprime(th), theta_range, 'UniformOutput', false);
    sigma_pprime_values = arrayfun(@(th) sigmapprime(th), theta_range, 'UniformOutput', false);
    
    % Extract components for plotting
    q1_values = cellfun(@(v) v(1), sigma_values);
    q2_values = cellfun(@(v) v(2), sigma_values);
    q1_prime_values = cellfun(@(v) v(1), sigma_prime_values);
    q2_prime_values = cellfun(@(v) v(2), sigma_prime_values);

    sigma_prime_0 = sigmaprime(0); 
    sigma_prime_1 = sigmaprime(1);

    for i = 1:numel(theta_range)
        % Extract current q and qdot from sigma and its derivative
        current_q = sigma_values{i};
        current_qdot = sigma_prime_values{i};

        % Numerically evaluate D, G, C using their respective function handles
        D_val = data.D(current_q);
        G_val = data.G(current_q);
        C_val = data.C([current_q; current_qdot]);  % Assume C depends on both q and qdot

        % Numerically calculate denominator, psi1, psi2
        denominator = B_perp * D_val * sigma_prime_values{i};
        psi1_num = -B_perp * G_val / denominator;
        psi2_num = -B_perp * (D_val * sigma_pprime_values{i} + C_val * sigma_prime_values{i}) / denominator;

        % Store results in cell arrays
        psi1_values{i} = psi1_num;
        psi2_values{i} = psi2_num;
    end

    % Convert cell arrays to arrays for interpolation
    psi1_array = cell2mat(psi1_values);  % Ensure this reshaping matches your data structure
    psi2_array = cell2mat(psi2_values);

    Deltadot_val = data.Deltadotfun([sigma_values{end}; sigma_prime_values{end}]);
    I_val = Deltadot_val(3:4, :);
    delta = (sigma_prime_0.' * I_val * sigma_prime_1) / (sigma_prime_0.' * sigma_prime_0);

    % Create interpolant functions
    psi1_interp = griddedInterpolant(theta_range, psi1_array, 'spline');
    psi2_interp = griddedInterpolant(theta_range, psi2_array, 'spline');
    % Deltadot_interp = griddedInterpolant(theta_range, Deltadot_array, 'spline');

    % Store these interpolants in the data structure passed to ODE
    data.Psi1_interp = psi1_interp;
    data.Psi2_interp = psi2_interp;

    % Set up the ODE options
    ops2 = odeset('RelTol', 1e-4, 'AbsTol', 1e-4);

    % Define the range for Theta and initial conditions
    Theta = linspace(0, 1, 1000);  % More points for better accuracy
    initial_conditions = [1; 0];  % Initial values for M and V

    % Solve the ODE
    [Theta, X] = ode45(@(theta, x) mass_potential(theta, x, data), Theta, initial_conditions, ops2);

    % Interpolate M and V for evaluation
    M = spline(Theta, X(:,1));
    V = spline(Theta, X(:,2));

    % Evaluate M and V at theta = 1 and find Vmax
    M_minus = ppval(M, 1);
    V_minus = ppval(V, 1);
    V_max = max(ppval(V, linspace(0, 1, 1000)));

    if visualization 
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
    % else 
    %     break;
    end 

end 

function xdot = mass_potential(theta, x, data)
    M = x(1);
    V = x(2);

    % Use interpolants to get Psi1 and Psi2 values at the current theta
    Psi1_val = data.Psi1_interp(theta);
    Psi2_val = data.Psi2_interp(theta);

    % Calculate derivatives of M and V
    Mdot = -2 * M * Psi2_val;
    Vdot = -Psi1_val * M;

    xdot = [Mdot; Vdot];
end