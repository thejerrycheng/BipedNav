function data = plot_phase_portrait(data, display)

    if display 
         % Define the grid for theta and omega
        theta_vals = linspace(0, 1, 100)';
        omega_vals = linspace(0, 20, 100)';  % Adjusted range based on typical system dynamics
        [Theta, Omega] = meshgrid(theta_vals, omega_vals);
        % Pre-calculate the energy grid for the static background
        % E_grid = zeros(size(Theta));
        % for i = 1:numel(Theta)
        %     th = Theta(i);
        %     om = Omega(i);
        %     M_val = ppval(data.M, th);
        %     V_val = ppval(data.V, th);
        %     E_grid(i) = 0.5 * M_val * om^2 + V_val;
        % end
    
        % Retrieve M and V values using ppval on the splines, adjusted for theta range [0, 1]
        M_vals = arrayfun(@(t) ppval(data.M, t), Theta);
        V_vals = arrayfun(@(t) ppval(data.V, t), Theta);
    
        % Calculate total energy E across the grid
        E = 1/2 * M_vals .* Omega.^2 + V_vals;

        figure;
        hold on;
        title('Phase Portrait with Energy Contours');
        xlabel('\theta');
        ylabel('\omega');
        set(gca, 'Color', 'w');  % Set axes background to white
    
        % Plot energy contours without filling
        contour(Theta, Omega, E, 50, 'LineWidth', 1);  % Non-filled contours for energy
        hColorbar = colorbar;  % Show a color bar indicating energy levels
        ylabel(hColorbar, 'Total Energy (E)');
    
        % Highlight a specific energy level
        Emax = max(V_vals(:));  % Assuming max potential energy as an important level
        [C, h] = contour(Theta, Omega, E, [Emax Emax], 'r', 'LineWidth', 2);
        clabel(C, h, 'Color', 'r', 'FontSize', 10, 'LabelSpacing', 400);
        legendEntry = sprintf('Critical Energy Level: E = %.2f', Emax);

        % % Draw a dashed red line connecting the start and end of the highest contour
        % if ~isempty(C)
        %     x = C(1, 2:C(2, 1)+1);  % x coordinates of the contour
        %     y = C(2, 2:C(2, 1)+1);  % y coordinates of the contour
        %     plot([x(1), x(end)], [y(1), y(end)], 'r--', 'LineWidth', 2);  % Dashed line
        % end
    
        % Optional: Plot simulated trajectories or known solutions
       % Loop through each simulation step
       if data.simulation_done 
           prev_ending_x = NaN;
           prev_ending_y = NaN;
           n = 0;
           for step = 1:data.number_steps
                if mod(step, 1) == 0  % Check if the step number is a multiple of 5 
                    n = n+1;
                    % Retrieve state from simulation data
                    q = data.X{step}(end, 1:2);  % Assuming q = [q1, q2]
                    qdot = data.X{step}(end, 3:4);  % Assuming qdot = [q1dot, q2dot]
        
                    % Convert q to theta
                    q1 = q(1);
                    q1_plus = data.q1_plus;  % From data or as a constant
                    q_tilde_1 = data.q_tilde_1;  % From data or as a constant
                    theta = (q1_plus - q1) / q_tilde_1;
                    omega = qdot(1) / q_tilde_1;  % Assuming qdot1 relates directly to theta dot

        
                    % Retrieve M and V values at current theta
                    M_val = ppval(data.M, theta);
                    V_val = ppval(data.V, theta);
        
                    % Calculate total energy E at current state
                    E_current = 0.5 * M_val * omega^2 + V_val;
        
                    % Plot current state energy contour
                    [C, h] = contour(Theta, Omega, E, [E_current E_current], 'r', 'LineWidth', 2);
                    clabel(C, h, 'Color', 'r', 'FontSize', 10, 'LabelSpacing', 400);
        
                    x_current = C(1, 2:C(2, 1)+1);  % x coordinates of the contour
                    y_current = C(2, 2:C(2, 1)+1);  % y coordinates of the contour
        
                    ending_x = x_current(end);
                    ending_y = y_current(end);
        
                    beginning_x = x_current(1);
                    beginning_y = y_current(1);
        
                    % Draw a line connecting the end of the previous contour to the beginning of the current one
                    if ~isnan(prev_ending_x) && step == 2
                        plot([prev_ending_x, beginning_x], [prev_ending_y, beginning_y], 'b--', 'LineWidth', 1);  % Dashed line
                    elseif ~isnan(prev_ending_x)
                        plot([prev_ending_x, beginning_x], [prev_ending_y, beginning_y], 'r--', 'LineWidth', 1);  % Dashed line
                    end
        
                    % Update the previous end coordinates to the current end coordinates
                    prev_ending_x = ending_x;
                    prev_ending_y = ending_y;
                end
            end
    
       end 
        % drawnow;
    
        % hold off;
        grid on;
        hold on; 
        % gird on;

        % % Set up the theta and omega grid for initial conditions
        theta_vals = linspace(0.1, 0.9, 10);
        omega_vals = linspace(1, 19, 10);
        [THETA, OMEGA] = meshgrid(theta_vals, omega_vals);

        % Initialize vectors to hold the derivatives
        U = zeros(size(THETA));
        V = zeros(size(OMEGA));

        % Calculate derivatives at each point in the grid
        for i = 1:numel(THETA)
            initialState = [THETA(i); OMEGA(i)];
            derivatives = constraintDynamics(0, initialState, data);
            U(i) = derivatives(1);
            V(i) = derivatives(2);
        end

        % Plot the phase portrait
        % figure;
        % quiver(THETA, OMEGA, U, V, 'r');  % Plot vectors
        title('Phase Portrait of Constraint Dynamics');
        xlabel('\theta');
        ylabel('\omega');
        axis tight;
        % hold off;
        drawnow;
        grid on;
        hold off;
    end 
end 


function dYdt = constraintDynamics(t, Y, data)
    theta = Y(1);  % First component of Y is theta
    omega = Y(2);  % Second component of Y is omega

    % Retrieve psi1 and psi2 values based on theta
    psi1_val = data.Psi1(theta);
    psi2_val = data.Psi2(theta);

    % Differential equations
    dthetadt = omega;
    domegadt = psi1_val + psi2_val * omega^2;

    % Return a column vector
    dYdt = [dthetadt; domegadt];
end
