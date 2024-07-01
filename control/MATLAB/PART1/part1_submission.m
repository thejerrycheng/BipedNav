%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ECE1658
%% FINAL PROJECT
%% VHC for walking acrobot
%% PARTIAL CODE TO GET STUDENTS STARTED ON THE PROJECT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear all
clc

number_steps=25;
symbolic_computations=1;
hybrid_limit_cycle = 1;
%% Physical parameters
l=1;
lc=0.5;
m=1;
Iz=1/12*m*l^2;
g=9.81;
psi=deg2rad(2);
incline_degrees = 0;
%% Control parameters
Kp=20^2; % Default Kp = 20^2; Kd = 20*2 
Kd=20*2;
data.Kp = Kp;
data.Kd = Kd;

%% Initialize the variables: 
beta = 0.316637;
beta = 1;
v1 = -0.894373 *10;
v2 = 1.9 *1.5; 
data.beta = beta; % store the numerical value into data
data.v1 = v1;
data.v2 = v2;
animationFileName = 'acrobot_animation_beta_1.mp4';  % Define the file name here

%% Symbolic computations
if symbolic_computations
    % Define symbolic variables
    fprintf('\n Initializing symbolic variables...\n')
    % syms l lc Iz m g real
    syms t q1 q2 x1 x2 q1dot q2dot x1dot x2dot tau real
    
    q=[q1;q2];
    x=[x1;x2]; 
    qbar=[q;x];

    qdot=[q1dot;q2dot];  
    xdot=[x1dot;x2dot]; 
    %% 
    qbardot=[qdot;xdot];
    
    fprintf('\n Symbolic model computation...\n')
    
    % Define centres of mass of two links
    rc1=x+lc*[cos(q1);sin(q1)];
    rc2=x+l*[cos(q1);sin(q1)]+lc*[cos(q1+q2);sin(q1+q2)];
    
    % Compute time derivatives of centres of mass
    rc1dot=jacobian(rc1,qbar)*qbardot;
    rc2dot=jacobian(rc2,qbar)*qbardot;
    
    % Define the total kinetic energy of the robot
    K=1/2*m*(rc1dot'*rc1dot+rc2dot'*rc2dot)+1/2*Iz*(q1dot^2+(q1dot+q2dot)^2);
    K=simplify(K);
    
    % Extract the square symmetric matrix of the kinetic energy
    Dbar=simplify(hessian(K,qbardot));
    
    % Extract the matrix of the kinetic energy of the pinned robot
    D = Dbar(1:2,1:2);
    
    % Define the potential energy of the pinnedrobot
    P = m*g*(lc*sin(q1)+l*sin(q1)+lc*sin(q1+q2));
    psi=deg2rad(incline_degrees);
    gvector=[cos(psi) -sin(psi);sin(psi) cos(psi)]*[0; -1];
    P=-(m*g*lc*[cos(q1);sin(q1)]+m*g*[l*cos(q1)+lc*cos(q1+q2);l*sin(q1)+lc*sin(q1+q2)])'*gvector;
    P=simplify(P);
    % Input matrix of pinned robot
    B=[0;1];
    B_perp = [1 0];
    
    % Computation of matrix C(q,qdot) of pinned robot
    C = sym(zeros(2,2));
    for i=1:2
        for j=1:2
            for k=1:2
                C(k,j)=C(k,j)+1/2*(diff(D(k,j),q(i))+diff(D(k,i),q(j))-diff(D(i,j),q(k)))*qdot(i);
            end
        end
    end
    
    % Computation of gradient of the potential for the pinned robot
    
    G = jacobian(P,q)';
    
    Dfun=matlabFunction(D,'Vars',{q});
    Gfun=matlabFunction(G,'Vars',{q});
    Cfun=matlabFunction(C,'Vars',{[q;qdot]});
    fprintf('\n Impact map computation...\n')
    
    %% Impact map
    
    % Relabelling map
    
    T=-eye(2)*q + [pi;2*pi];
    Delta2=[T;-eye(2)*qdot];
    
    % First impact map
    
    px=l*[cos(q1)+cos(q1+q2);sin(q1)+sin(q1+q2)];
    xo=sym(zeros(2,1));
    
    E=[jacobian(px,q), eye(2)];
    Deltaqdot=[eye(2),zeros(2,4)]*inv([Dbar -E';E zeros(2,2)])*...
        [Dbar*[eye(2);jacobian(xo,q)];zeros(2,2)];
    Delta1=eval([q;Deltaqdot*qdot]);
    Delta1=simplify(Delta1);
    
    % Composition of two maps
    Delta=simplify(subs(Delta2,[q;qdot],Delta1));
    Deltafun=matlabFunction(Delta,'Vars',{[q;qdot]});

   
    fprintf('\n Cache objects in data structure... \n ')
    data.D = Dfun;
    data.C = Cfun;
    data.G = Gfun;
    data.B = B;
    data.B_perp = B_perp;
    data.tau = tau; 

    save('walking_acrobot_model','D','Dfun','Cfun','Gfun','Deltafun','B');
else
    fprintf('\nLoading robot model...\n')
    load('walking_acrobot_model.mat');
end


%% HERE WRITE YOUR CODE FOR THE VHC DESIGN 


%% VHC Design precedure following the handout: 
% Define the constant variables for VHC design: 
q_minus = [(pi-beta)/2; pi+beta]; % Define q-
q_plus = [(pi+beta)/2; pi-beta]; % Define q+
q1_plus = q_plus(1);
q2_plus = q_plus(2);
q1_minus = q_minus(1);
q2_minus = q_minus(2);
q_tilde_1 = q1_plus - q1_minus; % Define q1~
q_tilde_2 = q2_plus - q2_minus; % Define q2~

% store the constant into the data structure: 
data.q1_plus = q1_plus;
data.q2_plus = q2_plus;
data.q1_minus = q1_minus;
data.q2_minus = q2_minus;
data.q_tilde_1 = q_tilde_1;
data.q_tilde_2 = q_tilde_2;

% Evaluate Deltaqdot for q_minus and qdot_minus
Deltaqdot = jacobian(Delta, qdot);
Deltaqdot_evaluated = double(subs(Deltaqdot, {q1, q2}, {q_minus(1), q_minus(2)}));

% Evaluate I matrix from the evaluated Deltaqdot
I_evaluated = Deltaqdot_evaluated(3:4, :);
data.I = I_evaluated;

% Display the resulting impact dynamics matrix I
disp('Computed Impact Dynamics Matrix I:');
disp(I_evaluated);

% Function f for hybrid invariance condition
I_11 = I_evaluated(1, 1);
I_12 = I_evaluated(1, 2);
I_21 = I_evaluated(2, 1);
I_22 = I_evaluated(2, 2);

% Define the function f(v1)
f = @(v1) (-q_tilde_1 * (-I_21 * q_tilde_1 + I_22 * v1)) / (-I_11 * q_tilde_1 + I_12 * v1);

% Define the polynomial degree
k = 6;

% Construct matrix V and vector b for the conditions
theta_vals = [0, 1, 0.5, 0, 1, 0.5];
powers = 0:k-1;  % Polynomial powers

% Construct the matrix based on the conditions
V = zeros(6, k);
for i = 1:6
    V(i, :) = theta_vals(i).^powers;
end
V(4, 1) = 0; % Adjust rows for derivative conditions
V(5, 1) = 0;
V(6, 1) = 0;
for j = 2:k
    V(4, j) = (j-1) * theta_vals(4)^(j-2);  % derivative at theta = 0
    V(5, j) = (j-1) * theta_vals(5)^(j-2);  % derivative at theta = 1
    V(6, j) = (j-1) * theta_vals(6)^(j-2);  % derivative at theta = 0.5
end

% Using f(v1) in the matrix setup for solving phi_a's coefficients
b = [q2_plus; q2_minus; pi; f(v1); v1; v2];

% Solve for the coefficients
a = V \ b;

% Display the resulting coefficients
fprintf('VHC coefficients: \n');
disp(a);
data.a = a;

%% HERE WE DEFINE THE FUNCTION phi_a AND ITS DERIVATIVES
% syms theta delta phi phiprime phipprime real

a=flip(a);  % Ensure coefficients are in the correct order for polyval
phi=@(theta) polyval(a,theta); % get q2 (parametrized using theta and polynomial function)
phiprime=@(theta) polyval(polyder(a),theta); % get q2'
phipprime=@(theta) polyval(polyder(polyder(a)),theta); % get q2''

% Include phi and its derivatives in the data structure
data.phi_fun = phi;
data.phiprime_fun = phiprime;
data.phipprime_fun = phipprime;

% Define sigma and its derivatives based on the project requirements -- the full parametrization for q = [q1; q2]
sigma = @(theta) [q1_plus - theta * q_tilde_1; phi(theta)];
sigmaprime = @(theta) [-q_tilde_1; phiprime(theta)];
sigmapprime = @(theta) [0; phipprime(theta)];

% Include sigma and its derivatives in the data structure
data.sigma_fun = sigma;
data.sigmaprime_fun = sigmaprime;
data.sigmapprime_fun = sigmapprime;

%% Plotting theta vs q, q', q'' under the constraint dynamics 
% Define theta range for plotting
theta_range = linspace(0, 1, 100);

% Evaluate sigma and its derivatives
sigma_values = arrayfun(@(th) sigma(th), theta_range, 'UniformOutput', false);
sigma_prime_values = arrayfun(@(th) sigmaprime(th), theta_range, 'UniformOutput', false);
sigma_double_prime_values = arrayfun(@(th) sigmapprime(th), theta_range, 'UniformOutput', false);

% Extract components for plotting
q1_values = cellfun(@(v) v(1), sigma_values);
q2_values = cellfun(@(v) v(2), sigma_values);
q1_prime_values = cellfun(@(v) v(1), sigma_prime_values);
q2_prime_values = cellfun(@(v) v(2), sigma_prime_values);
q1_pprime_values = cellfun(@(v) v(1), sigma_double_prime_values);
q2_pprime_values = cellfun(@(v) v(2), sigma_double_prime_values);

% Plotting
figure;
subplot(3, 1, 1);
plot(theta_range, q1_values, 'b-', theta_range, q2_values, 'r-');
title('Sigma Function - Joint Angles');
xlabel('Theta');
ylabel('Joint Angles');
legend('q1(theta)', 'q2(theta)');

subplot(3, 1, 2);
plot(theta_range, q1_prime_values, 'b--', theta_range, q2_prime_values, 'r--');
title('Sigma Prime Function - Derivatives of Joint Angles');
xlabel('Theta');
ylabel('Joint Velocity');
legend('q1^{.}(theta)', 'q2^{.}(theta)');

subplot(3, 1, 3);
plot(theta_range, q1_pprime_values, 'b--', theta_range, q2_pprime_values, 'r--');
title('Sigma Double Prime Function - Second Derivatives of Joint Angles');
xlabel('Theta');
ylabel('Joint Acceleration');
legend('q1^{..}(theta)', 'q2^{..}(theta)');

%% Convert the function to symbolic expression: 
syms theta thetadot %a1 a2 a3 a4 a5 a6
a_sym = double(sym(a));  % Convert numeric coefficients to symbolic
phi_sym = poly2sym(a_sym, theta);  % Convert to symbolic polynomial
phiprime_sym = diff(phi_sym, theta);  % First derivative
phipprime_sym = diff(phiprime_sym, theta);  % Second derivative

% Define sigma and its derivatives symbolically
sigma_sym = [q1_plus - theta * q_tilde_1; phi_sym];
sigmaprime_sym = [-q_tilde_1; phiprime_sym];
sigmapprime_sym = [0; phipprime_sym];

%% Control input calculated here: 
% Error term: -- Substitude theta in with an expression with q1
y = q2 - subs(phi_sym, theta, (q1_plus - q1)/q_tilde_1);

% H Jacobian of the error term about q: 
H = jacobian(y, q); % Used for later computing the control input 
Hfun=matlabFunction(H,'Vars',{q});
data.Hfun = Hfun;

% Define the time derivative of Jacobian
dHdt = sym(zeros(size(H))); % Initialize dH_dt as a zero matrix of the same size as H

for i = 1:size(H, 1)
    for j = 1:size(H, 2)
        for k = 1:length(q)
            % Partial derivative of H_ij with respect to q_k
            dH_ij_dqk = diff(H(i, j), q(k));

            % Add the contribution of this partial derivative times the derivative of q_k
            dHdt(i, j) = dHdt(i, j) + dH_ij_dqk * qdot(k);
        end
    end
end
% Create functions for dH/dt: 
dH_dt_fun = matlabFunction(dHdt, 'Vars', {[q; qdot]});
data.dH_dt_fun = dH_dt_fun;

%% HERE WRITE CODE TO TEST WHETHER YOUR VHC WORKS

%% (a) - (c): q1 - q2 plot:  
% Define theta range for plotting
theta_range = linspace(0, 1, 100);

% Evaluate sigma and its derivatives
sigma_values = arrayfun(@(th) sigma(th), theta_range, 'UniformOutput', false);
sigma_prime_values = arrayfun(@(th) sigmaprime(th), theta_range, 'UniformOutput', false);

% Extract components for plotting
q1_values = cellfun(@(v) v(1), sigma_values);
q2_values = cellfun(@(v) v(2), sigma_values);
q1_prime_values = cellfun(@(v) v(1), sigma_prime_values);
q2_prime_values = cellfun(@(v) v(2), sigma_prime_values);

% Plotting the VHC in the q1-q2 plane
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
grid on;
hold off;

%% (d) - Check for regular VHC condition that B_prep*D*sigma'(theta)!=0
% Define the theta range for evaluation
Theta = linspace(0, 1, 1000);  % Fine grid over the interval [0, 1]

% Define sigma_1 and sigma_2
Sigma = sigma(Theta);
sigma_1 = Sigma(1, :);
sigma_2 = Sigma(2, :);

% Project B to its orthogonal complement B_perp
B_perp = [1 0];  % Null space of B transpose gives orthogonal complement

% Evaluate the condition over theta
regularity_check = zeros(1, length(Theta))';
for i = 1:length(Theta)
    sigma_prime = sigmaprime(Theta(i));  % Construct the derivative vector at each theta
    % D_eval = double(subs(D, {q2}, {sigma_2(i)}));
    D_eval = Dfun(Sigma(:, i));
    projection = B_perp *  D_eval * sigma_prime;  % Project onto orthogonal complement of B
    regularity_check(i) = norm(projection);  % Store the norm of the projection
end

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

%% (e) Hybrid limit Cycle: 
% Using ODE to solve for M and V: 
% Example data or functional evaluations
Theta = linspace(0, 1, 15); % Captilized variables meaning it is in numerical values 
Phi = phi(Theta);
Sigma = sigma(Theta);

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

% Getting the expression for Psi1 and Psi2:  
denominator=simplify(B_perp*D*sigmaprime_sym);
psi1_sym=-B_perp*G/denominator;
psi2_sym=-B_perp*(D*sigmapprime_sym+subs(C,qdot,sigmaprime_sym)*sigmaprime_sym)/denominator;  
psi1_sym=simplify(subs(psi1_sym,q,sigma_sym));
psi2_sym=simplify(subs(psi2_sym,q,sigma_sym));

% Convert symbolic expressions to function handles for numerical integration
Psi1fun = matlabFunction(simplify(psi1_sym), 'Vars', {theta});
Psi2fun = matlabFunction(simplify(psi2_sym), 'Vars', {theta});

data.Psi1 = Psi1fun;
data.Psi2 = Psi2fun;

% % Integration Method to compute M and V: 
% Parameters
% Theta = linspace(0, 1, 100); % Using a finer grid for better accuracy
% theta0 = 0;
% M = zeros(1, length(Theta)); % Initialize M
% V = zeros(1, length(Theta)); % Initialize V
% 
% % Calculate M and V numerically using integrals
% for i = 1:length(Theta)
%     theta_eval = Theta(i);
% 
%     % Compute M(theta) using numerical integration for psi2
%     M(i) = exp(-2 * integral(@(t) Psi2fun(t), theta0, theta_eval));
% 
%     % Compute V(theta) using numerical integration for psi1
%     V(i) = -integral(@(t) Psi1fun(t), theta0, theta_eval);
% end
% 
% % Plot results
% figure;
% plot(Theta, M, 'DisplayName', 'Virtual Mass M(\theta)');
% hold on;
% plot(Theta, V, 'DisplayName', 'Virtual Potential V(\theta)');
% xlabel('\theta');
% ylabel('Values');
% title('Virtual Mass and Potential along the Constraint Manifold');
% legend show;

Theta = linspace(0, 1, 100); % Captilized variables meaning it is in numerical values 
Phi = phi(Theta);

phi_spline=spline(Theta,Phi);
phiprime_spline=fnder(phi_spline);
phipprime_spline=fnder(phiprime_spline);

% Creating spline representations
data.phi = phi_spline;
data.phiprime = phiprime_spline;
data.phipprime = phipprime_spline;

% Calculating M and V numerically using the ODE function: 
fprintf('\n...finding virtual mass and virtual potential...\n')
ops2=odeset('RelTol',1e-4,'AbsTol',1e-4);
[Theta,X]=ode45(@mass_potential,linspace(0,1,100)',[1;0],ops2,data);
M=spline(Theta,X(:,1));
V=spline(Theta,X(:,2));

% Evaluate M and V at theta = 1 and find Vmax
M_minus = ppval(M, 1);
V_minus = ppval(V, 1);
V_max = max(ppval(V, linspace(0, 1, 1000)));

% Compute delta
sigma_prime_0 = sigmaprime(0);
sigma_prime_1 = sigmaprime(1);
I = data.I; % Impact matrix
delta = (sigma_prime_0.' * I * sigma_prime_1) / (sigma_prime_0.' * sigma_prime_0);

% Stability conditions
condition1 = (delta^2 / M_minus) < 1;
condition2 = ((V_minus * delta^2) / (M_minus - delta^2)) + V_max < 0;

% Output the results
fprintf('Stability Condition 1 (0 < delta^2 / M_minus < 1): %d\n', condition1);
fprintf('Stability Condition 2 ((V_minus * delta^2) / (M_minus - delta^2) + V_max < 0): %d\n', condition2);

if condition1 && condition2
    disp('The VHC induces a stable hybrid limit cycle.');
else
    disp('The VHC does not induce a stable hybrid limit cycle. Check the implementation or parameter values.');
end

% Visualization for M and V vs theta
figure
plot(Theta,X(:,1));
hold on
plot(Theta,X(:,2));
legend('M','V')
drawnow;

%% Drawing the Phase Portrait:
figure;
hold on;
fprintf('\n...plotting phase portrait of constrained dynamics...\n')

Theta = linspace(-pi, pi, 100)';
ThetaDot = linspace(-20, 20, 100)';
[Theta, ThetaDot] = meshgrid(Theta, ThetaDot);

M_vals = arrayfun(@(t) ppval(M, wrapTo2Pi(t)), Theta);  % Using ppval since it is a spline
V_vals = arrayfun(@(t) ppval(V, wrapTo2Pi(t)), Theta);  % Same for V

% Calculate total energy E across the grid
E = 0.5 * M_vals .* ThetaDot.^2 + V_vals;

% Retrieve max and min values of potential energy for contour levels
VV = ppval(V, linspace(0, 2*pi, 1000));
Emax = max(VV);
Emin = min(VV);

% Plot energy contours
contour(Theta, ThetaDot, E, linspace(Emin, Emax, 10));
contour(Theta, ThetaDot, E, linspace(sqrt(Emax), sqrt(Emax)+10, 10).^2);
contour(Theta, ThetaDot, E, [Emax Emax], 'color', 'k', 'linewidth', 2);

xlabel('\theta');
ylabel('\theta dot');
axis tight;
title('Orbits of the constrained dynamics');

% If running a simulation and have q1, q1dot data
if exist('simulation', 'var') && simulation
    plot(wrapToPi(q1), q1dot, 'r');  % Simulation trajectory
    plot(wrapToPi(q1(1)), q1dot(1), 'ro');  % Starting point
end
drawnow;

%% NOW YOU CAN SIMULATE THE ROBOT. PLACE YOUR CONTROLLER INSIDE THE FUNCTION acrobot AT THE END OF THIS SCRIPT
ops= odeset('reltol',1e-7,'abstol',1e-7,'Events',@ground_impact);
dt=1/60; % 60 fps; time increment in simulations and animations

fprintf('\n Simulating...\n')

%% DEFINE THE INITIAL CONDITION [q0;qdot0];

% Define the initial condition as sigma0 and sigmaprime0: 
init_theta = 0.1; 
q0 = sigma(init_theta);
thetadot0 = delta * sqrt(-2*V_minus/(M_minus - delta^2));
qdot0 = sigmaprime(0)*thetadot0;

%% Adding disturbances to the initial condition:
% Limit cycle initial conditions
init_theta_lc = 0; % Small theta value on limit cycle
thetadot_lc = delta * sqrt(-2 * V_minus / (M_minus - delta^2));

% Perturbed initial conditions on the constraint manifold
% theta_range = linspace(init_theta_lc - 0.1, init_theta_lc + 0.1, 20); % 20 different starting thetas
theta = 0;

% for theta = theta_range
    % Calcualte the different initial conditions: 
    q0 = sigma(theta);
    thetadot0 = delta * sqrt(-2 * V_minus / (M_minus - delta^2));
    qdot0 = sigmaprime(0) * thetadot0;

    fprintf("The current initial theta is: %f", theta);

    %% Simulation starts here: 
    T=[];
    X=[];
    Te=[];
    Ie=[];
    Xe=[];
    post_impact_state=[q0;qdot0];
    % Initialize arrays to store time and torque data
    all_torque = [];
    all_time = [];
    
    for step=1:number_steps
        fprintf('\n...step %d...\n',step);
        [t,x,te,xe,ie]=ode45(@(t,x) acrobot(t,x,data),0:dt:10,post_impact_state,ops);

        % Collect torque data from the output (assuming it's included in x)
        torques = arrayfun(@(idx) calculateTorque(x(idx, :), data), 1:length(t));  % Assuming calculateTorque is a function you create

        % Collect all torques and corresponding times
        all_torque = [all_torque; torques'];
        
        % Making sure that the time increment as each iteration goes on 
        if ~isempty(all_time)
            t = t + all_time(end);
            all_time = [all_time; t];
        else 
            all_time = [all_time; t];
        end 

        % Application of the impact map
        impact_state=xe(end,:)';
        post_impact_state=Deltafun(impact_state);
        T{step}=t;
        X{step}=x;
        Ie{step}=ie;
        Te{step}=te;
        Xe{step}=xe;
    end    

    % Plot the generated torque with respect to time 
    figure;
    plot(all_time, all_torque);
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    title('Torque Generation Over Time');
    grid on;


    % % Simulate number_steps steps
    % for step = 1:number_steps
    %     fprintf('\n...step %d...\n', step);
    %     % Run the simulation from the current post-impact state
    %     [t, x, te, xe, ie] = ode45(@(t, x) acrobot(t, x, data), [0, 10], post_impact_state, ops);
    % 
    %     % Check if the events (impact events) have occurred
    %     if isempty(xe) || isempty(te)
    %         fprintf('Warning: No impact detected at step %d.\n', step);
    %         success = false;  % Set success to false if no events detected
    %         break; 
    %     else
    %         % Application of the impact map if events were detected
    %         impact_state = xe(end, :)';  % Use the last state where the event was detected
    %         post_impact_state = Deltafun(impact_state);  % Compute new post-impact state
    %     end
    % 
    %     % Store simulation results for this step
    %     T{step} = t;
    %     X{step} = x;
    %     Te{step} = te;
    %     Xe{step} = xe;
    %     Ie{step} = ie;
    % 
    % end

    % % Output the success status
    % if success
    %     fprintf('All simulations successful with events detected.\n');
    % else
    %     fprintf('One or more simulations failed to detect necessary events.\n');
    % end

    %% Animation of the simulation results
    % fprintf('\n Setting up animation...\n');
    % ref = 0; time_passed = 0; step = 1;
    % Axis = [-2 25 -0.5 1.5];
    % 
    % % Create a new figure with a specific size
    % figure;
    % set(gcf, 'Position', [100, 100, 1120, 840]); % Set figure to match the required video dimensions
    % 
    % % Set up the axes
    % axis(Axis);
    % axis equal;
    % hold on; % Hold the current axes to add multiple graphical objects
    % 
    % % Create a video writer object with 1 frame per second
    % v = VideoWriter(animationFileName, 'MPEG-4');
    % v.FrameRate = 60; % Set frame rate
    % open(v); % Open the file for writing
    % 
    % % Static line along x-axis from -2 to 25 at y = 0, named as ground_plane
    % ground_plane = plot([-2 25], [0 0], 'k-', 'LineWidth', 2); % Add the static line
    % 
    % % Add dynamic text for displaying time and steps
    % Time = text(3, 1.8, 'time= 0 secs, step= 1');  % Adjust position if needed
    % 
    % % Initial setup for the legs' animation
    % stance_leg = line([ref l*cos(q0(1))], [0 l*sin(q0(1))], 'color', 'red', 'linewidth', 2);
    % swing_leg = line([ref + l*cos(q0(1)) ref + l*cos(q0(1)) + l*cos(q0(1) + q0(2))],...
    %     [l*sin(q0(1)) l*sin(q0(1)) + l*sin(q0(1) + q0(2))], 'linewidth', 2);
    % 
    % fprintf('\nAnimation is ready...\n')
    % pause; % Pause to view the initial setup
    % animation_slowdown_factor = 0.1; % >1 means slow down
    % 
    % for step = 1:length(Ie)
    %     t = T{step};
    %     x = X{step};
    %     xe = Xe{step};
    %     xe = xe(end, :);
    %     for k = 2:length(t)
    %         drawnow;
    %         q = x(k, 1:2)';
    %         xdata1 = [ref ref + l*cos(q(1))];
    %         xdata2 = [ref + l*cos(q(1)) ref + l*cos(q(1)) + l*cos(q(1) + q(2))];
    %         ydata1 = [0 l*sin(q(1))];
    %         ydata2 = [l*sin(q(1)) l*sin(q(1)) + l*sin(q(1) + q(2))];
    % 
    %         set(stance_leg, 'xdata', xdata1, 'ydata', ydata1);
    %         set(swing_leg, 'xdata', xdata2, 'ydata', ydata2);
    %         set(Time, 'String', ['time= ', num2str(round(time_passed + t(k), 1)), ' secs,', ' step= ', num2str(step)]);
    % 
    %         % Capture the frame
    %         frame = getframe(gcf);
    %         writeVideo(v, frame);
    %     end
    %     time_passed = time_passed + t(end);
    %     ref = ref + l * (cos(xe(1)) + cos(xe(1) + xe(2)));
    % end
    % 
    % % Close the video file
    % close(v);
    % fprintf('\nAnimation saved to acrobot_animation.mp4\n');

    fprintf('\n Setting up animation...\n');
ref = 0; time_passed = 0; step = 1;
Axis = [-2 25 -0.5 1.5];

% Create a new figure with a specific size
figure;
set(gcf, 'Position', [100, 100, 1120, 840]); % Set figure to match the required video dimensions

% Set up the axes
axis(Axis);
axis equal;
hold on; % Hold the current axes to add multiple graphical objects

% Ensure that the axes limits are set and held constant
set(gca, 'XLim', [-2 25], 'YLim', [-0.5 1.5]);

% Create a video writer object with 1 frame per second
animationFileName = 'acrobot_animation.mp4'; % Define the file name variable at the top
v = VideoWriter(animationFileName, 'MPEG-4');
v.FrameRate = 60; % Set frame rate
open(v); % Open the file for writing

% Static line along x-axis from -2 to 25 at y = 0, named as ground_plane
ground_plane = plot([-2 25], [0 0], 'k-', 'LineWidth', 2); % Add the static line

% Add dynamic text for displaying time and steps
Time = text(3, 1.8, 'time= 0 secs, step= 1');  % Adjust position if needed

% Initial setup for the legs' animation
stance_leg = line([ref l*cos(q0(1))], [0 l*sin(q0(1))], 'color', 'red', 'linewidth', 2);
swing_leg = line([ref + l*cos(q0(1)) ref + l*cos(q0(1)) + l*cos(q0(1) + q0(2))],...
    [l*sin(q0(1)) l*sin(q0(1)) + l*sin(q0(1) + q0(2))], 'linewidth', 2);

fprintf('\nAnimation is ready...\n')
pause; % Pause to view the initial setup
animation_slowdown_factor = 0.1; % >1 means slow down

for step = 1:length(Ie)
    t = T{step};
    x = X{step};
    xe = Xe{step};
    xe = xe(end, :);
    for k = 2:length(t)
        drawnow;
        q = x(k, 1:2)';
        xdata1 = [ref ref + l*cos(q(1))];
        xdata2 = [ref + l*cos(q(1)) ref + l*cos(q(1)) + l*cos(q(1) + q(2))];
        ydata1 = [0 l*sin(q(1))];
        ydata2 = [l*sin(q(1)) l*sin(q(1)) + l*sin(q(1) + q(2))];

        set(stance_leg, 'xdata', xdata1, 'ydata', ydata1);
        set(swing_leg, 'xdata', xdata2, 'ydata', ydata2);
        set(Time, 'String', ['time= ', num2str(round(time_passed + t(k), 1)), ' secs,', ' step= ', num2str(step)]);

        % Capture the frame
        frame = getframe(gcf);
        writeVideo(v, frame);
    end
    time_passed = time_passed + t(end);
    ref = ref + l * (cos(xe(1)) + cos(xe(1) + xe(2)));
end

% Close the video file
close(v);
fprintf('\nAnimation saved to acrobot_animation.mp4\n');

% end

%% FUNCTIONS
function xdot = acrobot(t, x, data)
    % Unpack the state variables
    q1 = x(1);
    q2 = x(2);
    q1dot = x(3);
    q2dot = x(4);
    q = [q1; q2];
    qdot = [q1dot; q2dot];

    % Controller gains and system parameters from 'data'
    Kp = data.Kp;
    Kd = data.Kd;
    D = data.D(q);
    C = data.C([q;qdot]);
    G = data.G(q);
    B = data.B;
    tau = data.tau;

    % VHC parameters and function evaluations
    q1_plus = data.q1_plus;
    q_tilde_1 = data.q_tilde_1;
    theta = (q1_plus - q1) / q_tilde_1;
    thetadot = -q1dot / q_tilde_1;

    phi = data.phi_fun(theta);
    phiprime = data.phiprime_fun(theta);
    phipprime = data.phipprime_fun(theta);

    % Define the output y and its derivatives
    y = q2 - phi;
    ydot = q2dot - phiprime * thetadot;

    % Compute Jacobian H and its derivative dH_dt
    H = data.Hfun(q);  % H evaluated at the current state
    dH_dt = data.dH_dt_fun([q; qdot]);  % dH_dt evaluated at the current state

    % Control inputs
    tau = inv(H*inv(D)*B) * (H*inv(D)*(C*qdot+G) - Kp*sin(y) - Kd*H*qdot - dH_dt*qdot);  % Solve for control input tau
    % tau = 0;

    % Compute qddot given tau as the feedback linearlized input 
    qddot = inv(D) * (B * tau - C * qdot - G);

    xdot = [qdot; qddot];
end

function [value,isterminal,direction]=ground_impact(t,x)
    q1=x(1);
    q2=x(2);
    % impact occurs when q2 = -2*q1+2*pi
    value = q2 + 2*q1-2*pi;
    
    % We exclude the scuffing point from the impact conditions
    if abs(q1-pi/2)<0.01
        isterminal=0;
    else
        isterminal=1;
    end
    
    % We distinguish between impact on S^+ or S^- by changing the way in which
    % ode45 monitors the zero crossing
    if abs(q1)<pi/2
        direction=-1;
    else
        direction=1;
    end
    % direction = 0;
end

function xdot = mass_potential(theta, x, data)
    M = x(1);
    V = x(2);
    phi = ppval(data.phi, theta);
    phiprime = ppval(data.phiprime, theta);
    phipprime = ppval(data.phipprime, theta);
    Mdot = -2 * M * data.Psi2(theta);
    Vdot = -data.Psi1(theta) * M;
    xdot = [Mdot; Vdot];
end

function tau = calculateTorque(x, data)
    q1 = x(1);
    q2 = x(2);
    q1dot = x(3);
    q2dot = x(4);
    q = [q1; q2];
    qdot = [q1dot; q2dot];

    % Controller gains and system parameters from 'data'
    Kp = data.Kp;
    Kd = data.Kd;
    D = data.D(q);
    C = data.C([q;qdot]);
    G = data.G(q);
    B = data.B;

    % VHC parameters and function evaluations
    q1_plus = data.q1_plus;
    q_tilde_1 = data.q_tilde_1;
    theta = (q1_plus - q1) / q_tilde_1;
    thetadot = -q1dot / q_tilde_1;

    phi = data.phi_fun(theta);
    phiprime = data.phiprime_fun(theta);
    phipprime = data.phipprime_fun(theta);

    % Define the output y and its derivatives
    y = q2 - phi;
    ydot = q2dot - phiprime * thetadot;

    % Compute Jacobian H and its derivative dH_dt
    H = data.Hfun(q);  % H evaluated at the current state
    dH_dt = data.dH_dt_fun([q; qdot]);  % dH_dt evaluated at the current state

    % Control inputs
    tau = inv(H*inv(D)*B) * (H*inv(D)*(C*qdot+G) - Kp*sin(y) - Kd*H*qdot - dH_dt*qdot);  % Solve for control input tau
end


