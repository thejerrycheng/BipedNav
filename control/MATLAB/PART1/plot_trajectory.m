function plot_trajectory(data)

    Theta = linspace(0, 1, 12); % Captilized variables meaning it is in numerical values 
    Phi = data.phi_fun(Theta);
    Sigma = data.sigma_fun(Theta);
    l = data.l;
    
    % Visualize the robot configurations under the constraint set: 
    figure;
    hold on; 
    axis([-1 1 -0.5 1.5]);
    axis equal;
    Q1 = Sigma(1, :);
    Q2 = Sigma(2, :);
    for i = 1:length(Q1)
        Q = [Q1(i); Q2(i)];
        stance_leg = line([0 l*cos(Q(1))], [0 l*sin(Q(1))], 'color', 'red', 'linewidth', 2);
        swing_leg = line([l*cos(Q(1)) l*cos(Q(1)) + l*cos(Q(1) + Q(2))],...
            [l*sin(Q(1)) l*sin(Q(1)) + l*sin(Q(1) + Q(2))], 'linewidth', 2);
    end 
    
    plot(l*cos(Q1)+l*cos(Q1+Q2), l*sin(Q1)+l*sin(Q1+Q2),'g','linewidth',2)
    title('acrobot configurations on the constraint set')
    drawnow;

end 




