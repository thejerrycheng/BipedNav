%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ECE1658
%% FINAL PROJECT
%% VHC for walking acrobot
%% PARTIAL CODE TO GET STUDENTS STARTED ON THE PROJECT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear all
clc

number_steps=2;
data.number_steps = number_steps;
symbolic_computations=1;
hybrid_limit_cycle = 1;
data.simulation_done = 0;

%% Physical parameters
l=1; data.l = l;
lc=0.5; data.lc = lc;
m=1; data.m = m;
Iz=1/12*m*l^2; data.Iz = Iz; 
g=9.81; data.g = g;
psi=deg2rad(0); data.psi = psi; 
incline_degrees = 0; data.incline_degrees = incline_degrees;
%% Control parameters
Kp=40^2; % Default Kp = 20^2; Kd = 20*2 
Kd=20*2;
Kdd = 1;
data.Kp = Kp;
data.Kd = Kd;
data.Kdd = Kdd;

%% Initialize the variables: 
beta = 0.316637;
v1 = -0.894373 ;
v2 = 1.9 ; 
data.beta = beta; % store the numerical value into data
data.v1 = v1;
data.v2 = v2;
animationFileName = 'acrobot_animation_beta_pi_4.mp4';  % Define the file name here

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
    Deltaqdot=[eye(2),zeros(2,4)]*inv([Dbar -E';E zeros(2,2)])*[Dbar*[eye(2);jacobian(xo,q)];zeros(2,2)];
    Delta1=eval([q;Deltaqdot*qdot]);
    Delta1=simplify(Delta1);
    
    % Composition of two maps
    Delta=simplify(subs(Delta2,[q;qdot],Delta1));
    Deltafun=matlabFunction(Delta,'Vars',{[q;qdot]});

    data.Delta = Delta;
    data.Deltafun = Deltafun;
   
    fprintf('\n Cache objects in data structure... \n ')
    data.D = Dfun;
    data.C = Cfun;
    data.G = Gfun;
    data.B = B;
    data.B_perp = B_perp;
    data.tau = tau; 

    data.D_sym = D;
    data.G_sym = G;
    data.C_sym = C; 

    save('walking_acrobot_model','D','Dfun','Cfun','Gfun','Deltafun','B');
else
    fprintf('\nLoading robot model...\n')
    load('walking_acrobot_model.mat');
end


%% HERE WRITE YOUR CODE FOR THE VHC DESIGN 


%% VHC Design precedure following the handout: 
% Define the constant variables for VHC design: 
q_minus = [(pi-beta)/2 + psi; pi+beta]; % Define q-
q_plus = [(pi+beta)/2 + psi; pi-beta]; % Define q+
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
Deltadotfun=matlabFunction(Deltaqdot,'Vars',{[q;qdot]});
data.Deltadotfun = Deltadotfun;

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
data.f = f;

%% PART 1: Linear Optimization Approach Finding a: 
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
a=flip(a);  % Ensure coefficients are in the correct order for polyval
data.a = a;

%% HERE WE DEFINE THE FUNCTION phi_a AND ITS DERIVATIVES
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
plot_VHC_curve(data);

%% (d) - Check for regular VHC condition that B_prep*D*sigma'(theta)!=0
display_transversality=1;
transversality_check(data, display_transversality);

%% (e) Hybrid limit Cycle: 
% Visualization of the dynamics constraint -- swing leg trajectory plotting: 
plot_trajectory(data);

% Constrained dynamics Getting the expression for Psi1 and Psi2:  
display_VM_graph = 1;
data = constraint_dynamics_check(data, display_VM_graph); % Help calculate M, V, delta as well

%% NOW YOU CAN SIMULATE THE ROBOT. PLACE YOUR CONTROLLER INSIDE THE FUNCTION acrobot AT THE END OF THIS SCRIPT
ops= odeset('reltol',1e-7,'abstol',1e-7,'Events',@ground_impact);
dt=1/60; % 60 fps; time increment in simulations and animations

fprintf('\n Simulating...\n')

% Adding disturbances to the initial condition:
% Limit cycle initial conditions

delta = data.delta;
M_minus = data.M_minus;
V_minus = data.V_minus;
V_max = data.V_max;

theta = 0; 
q0 = sigma(0) + [0.1; 0.1];
thetadot0 = delta * sqrt(-2*V_minus/(M_minus - delta^2));
qdot0 = 1.2*sigmaprime(0) * thetadot0;
% qdot0 = [0;0];

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

%% Plotting the Phase Portrait for the disturbance:
data.simulation_done = 1;
data.T = T;
data.X = X;
data.Te = Te;
data.Xe = Xe; 
display = 1;
data = plot_phase_portrait(data, display);

%% 
%% Animation Starts here... 

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
# Initialize symbolic variables
l, lc, Iz, m, g = sp.symbols('l lc Iz m g', real=True)
t, q1, q2, x1, x2, q1dot, q2dot, x1dot, x2dot, tau = sp.symbols('t q1 q2 x1 x2 q1dot q2dot x1dot x2dot tau', real=True)

q = sp.Matrix([q1, q2])
x = sp.Matrix([x1, x2])
qbar = sp.Matrix([q1, q2, x1, x2])
qdot = sp.Matrix([q1dot, q2dot])
xdot = sp.Matrix([x1dot, x2dot])
qbardot = sp.Matrix([q1dot, q2dot, x1dot, x2dot])

# Define centers of mass of two links
rc1 = x + lc * sp.Matrix([sp.cos(q1), sp.sin(q1)])
rc2 = x + l * sp.Matrix([sp.cos(q1), sp.sin(q1)]) + lc * sp.Matrix([sp.cos(q1 + q2), sp.sin(q1 + q2)])

# Compute time derivatives of centers of mass
rc1dot = rc1.jacobian(qbar) * qbardot
rc2dot = rc2.jacobian(qbar) * qbardot

# Define the total kinetic energy of the robot
K = 1/2 * m * (rc1dot.T * rc1dot + rc2dot.T * rc2dot)[0] + 1/2 * Iz * (q1dot**2 + (q1dot + q2dot)**2)
K = sp.simplify(K)

# Extract the square symmetric matrix of the kinetic energy
Dbar = sp.Matrix(sp.hessian(K, qbardot))

# Extract the matrix of the kinetic energy of the pinned robot
D = Dbar[:2, :2]

# Define the potential energy of the pinned robot
P = m * g * (lc * sp.sin(q1) + l * sp.sin(q1) + lc * sp.sin(q1 + q2))

# Input matrix of pinned robot
B = sp.Matrix([0, 1])

# Computation of matrix C(q,qdot) of pinned robot
C = sp.zeros(2, 2)
for i in range(2):
    for j in range(2):
        for k in range(2):
            C[k, j] += 1/2 * (sp.diff(D[k, j], q[i]) + sp.diff(D[k, i], q[j]) - sp.diff(D[i, j], q[k])) * qdot[i]

# Computation of gradient of the potential for the pinned robot
G = P.jacobian(q).T

# Computation of qddot (pinned robot)
qddot = sp.simplify(D.inv() * (-C * qdot - G + B * tau))

# Convert to numerical functions
l_val, lc_val, m_val, Iz_val, g_val, tau_val = 1, 0.5, 1, 1/12, 9.81, 0
params = {l: l_val, lc: lc_val, m: m_val, Iz: Iz_val, g: g_val, tau: tau_val}


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

function [value,isterminal,direction] = ground_impact(t,x)
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
    Kdd = data.Kdd;
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
    tau = inv(H*inv(D)*B) * (H*inv(D)*(C*qdot+G) - Kp*sin(y) - Kd*H*qdot - Kdd*dH_dt*qdot);  % Solve for control input tau
end


