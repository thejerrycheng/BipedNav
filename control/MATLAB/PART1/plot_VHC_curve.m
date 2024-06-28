function plot_VHC_curve(data)

    beta = data.beta;
    a = data.a;
    
    % Define dynamic constants
    q_minus = [(pi-beta)/2; pi+beta]; q_plus = [(pi+beta)/2; pi-beta]; % Define q-, q+
    q1_plus = q_plus(1); q2_plus = q_plus(2); q1_minus = q_minus(1); q2_minus = q_minus(2);
    q_tilde_1 = q1_plus - q1_minus; q_tilde_2 = q2_plus - q2_minus; % Define q1~; q2~ 
    
    % Getting the updated Phi and Sigma function to evaluate q, qdot, qddot
    phi=@(theta) polyval(a,theta); % 
    phiprime=@(theta) polyval(polyder(a),theta); % get q2'
    phipprime=@(theta) polyval(polyder(polyder(a)),theta); 
    sigma = @(theta) [q1_plus - theta * q_tilde_1; phi(theta)];
    sigmaprime = @(theta) [-q_tilde_1; phiprime(theta)];
    sigmapprime = @(theta) [0; phipprime(theta)];
    
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
    
    %% Plotting the VHC in the q1-q2 plane
    figure;
    plot(q1_values, q2_values, 'b-', 'LineWidth', 2);
    hold on;
    xlabel('q1 (Joint Angle 1)');
    ylabel('q2 (Joint Angle 2)');
    title('VHC Plot in q1-q2 Plane');
    
    % Additional line from (pi, 0) to (0, 2pi)
    plot([pi, 0], [0, 2*pi], 'k-', 'LineWidth', 1.5, 'DisplayName', 'Additional Line');
    
    % Mark q+ and q- on the plot
    plot(q1_plus, q2_plus, 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'q^+');
    plot(q1_minus, q2_minus, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'q^-');
    
    % Define the scuffing point q_bar as the midpoint in the trajectory
    q_bar = sigma(0.5);  % Assuming theta=0.5 corresponds to index 50
    plot(q_bar(1), q_bar(2), 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'q^{\bar{}} (Scuffing Point)');
    
    % Add tangent lines at critical theta values
    critical_thetas = [0, 0.5, 1];
    critical_colors = ['k', 'm', 'g'];  % Colors for each critical theta
    for i = 1:length(critical_thetas)
        theta_eval = critical_thetas(i);
        q_eval = sigma(theta_eval);
        q1 = q_eval(1);
        q2 = q_eval(2);
        q_prime = sigmaprime(theta_eval);
        q1_prime = q_prime(1);
        q2_prime = q_prime(2);
    
        % Determine the line length for tangents
        line_length = 0.1;  
        tangent_x = [q1, q1 + line_length * q1_prime];
        tangent_y = [q2, q2 + line_length * q2_prime];
    
        % Determine the scale for arrow length, adjust as necessary for visibility
        scale = 0.1;  % Scale factor for arrow length
    
        % Plot tangent arrow using quiver
        quiver(q1, q2, scale * q1_prime, scale * q2_prime, 'Color', critical_colors(i), 'MaxHeadSize', 0.5, 'AutoScale', 'off', 'LineWidth', 1.5);
    end
    
    % Set axis limits and labels
    xlim([0 pi+0.2]);
    ylim([0 2*pi+0.4]);
    
    % Add legends and grid
    legend('VHC Curve', 'Tangent at \theta=0', 'Tangent at \theta=0.5', 'Tangent at \theta=1');
    % grid on;
    % hold off;

end 

