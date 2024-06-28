function transversality_check(data, display_trans)

    % Define the transversality function  
    transversality = @(theta) data.B_perp*data.D(data.sigma_fun(theta))*data.sigmaprime_fun(theta) ;
    % Evaluate the condition over theta
    Theta = linspace(0, 1, 1000);  % Fine grid over the interval [0, 1]
    regularity_check = zeros(1, length(Theta))';
    
    for i = 1:length(Theta)
        regularity_check(i) = norm(transversality(Theta(i)));  % Store the norm of the projection
    end

    if display_trans
        % Check for any zero crossings
        if any(regularity_check == 0)
            disp('The VHC is not regular for all theta in [0, 1].');
        else
            disp('The VHC is regular for all theta in [0, 1].');
        end
        
        % Plot to visualize
        figure;
        plot(Theta, regularity_check);
        xlabel('\theta');
        ylabel('|B^{\perp} D \sigma''(\theta)|');
        title('Regularity Check of the VHC');
        grid on;
    end 
end 