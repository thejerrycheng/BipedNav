%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ECE1658
%% Assignment 1 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear all
clc

% Define symbolic variables
fprintf('\n Initializing symbolic variables...\n')
syms t x q1 q2 q3 q4 q5 x1 x2 q1dot q2dot q3dot q4dot q5dot x1dot x2dot tau real 
% syms l1 l2 l3 m1 m2 m3 m3 m4 m5 m6 g real

%% Physical parameters
% Define numerical variables
l1 = 0.5; 
l2 = 0.5; 
l3 = 0.3; % lengths
m1 = 0.05; 
m2 = 0.5; 
m3 = 0.3; 
m4 = 0.5; 
m5 = 0.05; % masses
m6 = 0.5;
g = 9.81; % gravitational acceleration

q = [q1; q2; q3; q4; q5];
% x = [x1; x2];
qbar=[q;x1;x2];

qdot = [q1dot; q2dot; q3dot; q4dot; q5dot];
qbardot=[qdot;x1dot; x2dot];

fprintf('\n Symbolic model computation...\n')

% Define centres of mass of two links
rc1=[x1; x2];
rc2=[x1; x2]+l1*[cos(q1);sin(q1)];
rc3=[x1; x2]+l1*[cos(q1);sin(q1)]+l2*[cos(q1+q2);sin(q1+q2)];
rc4=[x1; x2]+l1*[cos(q1);sin(q1)]+l2*[cos(q1+q2);sin(q1+q2)]+l2*[cos(q1+q2+q3);sin(q1+q2+q3)];
rc5=[x1; x2]+l1*[cos(q1);sin(q1)]+l2*[cos(q1+q2);sin(q1+q2)]+l2*[cos(q1+q2+q3);sin(q1+q2+q3)]+l1*[cos(q1+q2+q3+q4);sin(q1+q2+q3+q4)];
rc6=[x1; x2]+l1*[cos(q1);sin(q1)]+l2*[cos(q1+q2);sin(q1+q2)]+l3*[cos(q1+q2+q5);sin(q1+q2+q5)];

% Compute time derivatives of centres of mass
rc1dot=jacobian(rc1,qbar)*qbardot;
rc2dot=jacobian(rc2,qbar)*qbardot;
rc3dot=jacobian(rc3,qbar)*qbardot;
rc4dot=jacobian(rc4,qbar)*qbardot;
rc5dot=jacobian(rc5,qbar)*qbardot;
rc6dot=jacobian(rc6,qbar)*qbardot;

% Define the total kinetic energy of the robot
K= 1/2*m1*(rc1dot'*rc1dot) + 1/2*m2*(rc2dot'*rc2dot)+1/2*m3*(rc3dot'*rc3dot)+1/2*m4*(rc4dot'*rc4dot)+...
    1/2*m5*(rc5dot'*rc5dot)+1/2*m6*(rc6dot'*rc6dot);

% Extract the square symmetric matrix of the kinetic energy
Dbar=simplify(hessian(K,qbardot));

% Extract the matrix of the kinetic energy of the pinned robot
D = Dbar(1:5,1:5);

% Define the potential energy of the pinnedrobot
P = m1*g*x2 + m2*g*l1*(sin(q1)+x2) + m3*g*(l2*sin(q1+q2)+l1*sin(q1)+x2) + m4*g*(l2*sin(q1+q2)+l1*sin(q1)+l2*sin(q1+q2+q3)+x2) + ...
    m5*g*(l2*sin(q1+q2)+l1*sin(q1)+l2*sin(q1+q2+q3)+l1*sin(q1+q2+q3+q4)+x2) + m6*g*(l1*sin(q1)+l2*sin(q1+q2)+l3*sin(q1+q2+q5)+x2);

% Input matrix of pinned robot
B = [0 0 0 0;    % Corresponds to q1, which is unactuated
     1 0 0 0;    % Corresponds to q2
     0 1 0 0;    % Corresponds to q3
     0 0 1 0;    % Corresponds to q4
     0 0 0 1];   % Corresponds to q5

% Computation of matrix C(q,qdot) of pinned robot
C = sym(zeros(5,5));
for i=1:5
    for j=1:5
        for k=1:5
            C(k,j)=C(k,j)+1/2*(diff(D(k,j),q(i))+diff(D(k,i),q(j))-diff(D(i,j),q(k)))*qdot(i);
        end
    end
end

% Computation of gradient of the potential for the pinned robot

G = jacobian(P,q)';

% Computation of qddot (pinned robot)

qddot = inv(D)*(-C*qdot-G+B*tau);

fprintf('\n Impact map computation...\n')

%% 
% First impact map for 5-link robot
% Define the position vector px for the foot experiencing impact
px = [l1*cos(q1) + l2*cos(q1 + q2) + l2*cos(q1 + q2 + q3) + l1*cos(q1 + q2 + q3 + q4); % Adjust as needed
      l1*sin(q1) + l2*sin(q1 + q2) + l2*sin(q1 + q2 + q3) + l1*sin(q1 + q2 + q3 + q4)]; % Adjust as needed

% E = [jacobian(p, q), zeros(2,2)];
E = simplify(jacobian(px, qbar));

%% Creating the data structure
% Define the functions to turn the symbolic expression into function to get numerical values: 
% Assuming D, Dbar, px, E, C, G are already defined symbolic expressions

% Create functions from the symbolic expressions
Dfun = matlabFunction(D, 'Vars', [q1; q2; q3; q4; q5]);
Dbarfun = matlabFunction(Dbar, 'Vars', [q1; q2; q3; q4; q5]);
pxfun = matlabFunction(px, 'Vars', [q1; q2; q3; q4; q5]);
Efun = matlabFunction(E, 'Vars', [q1; q2; q3; q4; q5]);
Cfun = matlabFunction(C, 'Vars', {[q1; q2; q3; q4; q5]; [q1dot; q2dot; q3dot; q4dot; q5dot]});
Gfun = matlabFunction(G, 'Vars', [q1; q2; q3; q4; q5]);

% Example of how to use these functions:
% q_numerical = [value1; value2; value3; value4; value5];
% D_evaluated = Dfun(q_numerical);

fprintf('\n Define the data structure for the variables... \n ')
% Create the structure array
data = struct('D', Dfun, 'Dbar', Dbarfun, 'E', Efun, 'C', Cfun, 'G', Gfun, 'B', B);


%% Control: 

%% part a
% Define numerical variables for reference joint angles
q2ref = pi/6; 
q3ref = pi + pi/6; 
q4ref = -pi/6; 
q5ref = -pi/6;

% Place them in a column vector qref
qref = [q2ref; q3ref; q4ref; q5ref];

% Define the 4x5 matrix H
H = [0 1 0 0 0;
     0 0 1 0 0;
     0 0 0 1 0;
     0 0 0 0 1];

% Determine control gains Kp and Kd
% For a characteristic equation lambda^2 + Kd*lambda + Kp = 0
% with roots at -20, the coefficients are:
desired_roots = [-20, -20];
poly_coeffs = poly(desired_roots);
Kd = poly_coeffs(2); % Coefficient of lambda
Kp = poly_coeffs(3); % Constant term

% Add variables to the data structure
data.qref = qref;
data.Kp = Kp;
data.Kd = Kd;
data.H = H;

%% part b

%% symbolic definition: 

% Define symbolic variables
syms theta real
B_perp = sym([1, 0, 0, 0, 0]);

% Define sigma(theta) symbolically
sigma = [theta; q2ref; q3ref; q4ref; q5ref];
sigma_prime = diff(sigma, theta); % this is essentially [1;0;0;0;0] 
D_sigma = subs(D, [q1; q2; q3; q4; q5], sigma);
expression = B_perp * D_sigma * sigma_prime;
simplified_expression = simplify(expression);

% Calculating tau:
% Dfun = data.D; % Function handle for D matrix
% Cfun = data.C; % Function handle for C matrix
% Gfun = data.G; % Function handle for gradient of potential energy
% D = Dfun(q);
% D_inv = inv(D);
% C = Cfun(q, qdot);
% G = Gfun(q);
% tau = (H * D_inv * B)^(-1) * (H * D_inv * (C * qdot + G) - Kp * sin(H * q - qref) - Kd * H * qdot);
% 
% % Compute qddot
% qddot = D_inv * (-C * qdot - G + B * tau);

%% pass into the defined funciton fro biped robot to comput xdot: 
x = [q;qdot];
xdot = biped(t, x, data);
qdot = xdot(1:5);
qddot = xdot(6:end);

%% Impact map for 5-link walking robot

%% Optimized Code: 
% Precompute Inverse Matrix Once

Delta = impact_map(x, data);
Deltafun = matlabFunction(Delta, 'Vars', {[q; qdot]});

% invMatrix = inv([Dbar, -E'; E, zeros(2, 2)]);
% 
% % Delta 1: 
% Deltaqdot = [eye(5), zeros(5, 4)] * invMatrix * [Dbar * [eye(5); zeros(2, 5)]; zeros(2, 5)];
% Delta1 = [q; Deltaqdot * qdot];
% 
% % Delta 2: 
% % Relabelling map for 5 joints
% R = [1,1,1,1,0; 
%      0,0,0,-1,1; 
%      0,0,-1,0,0; 
%      0,-1,0,0,0; 
%      0,0,-1,0,1]; 
% 
% d = [-3*pi; 2*pi; 2*pi; 2*pi; pi];
% q_tilde = R*q + d; % Relabelling 
% Delta2 = [q_tilde; R*qdot];
% 
% % Total Delta: 
% % Composition of two maps
% Delta = simplify(subs(Delta2, [q; qdot], Delta1));
% 


fprintf('\n Setting up simulations...\n')


%% Setting up simulations

% Ensure that all the variables and parameters from the previous model are initialized here

% ODE of pinned acrobot (using dynamics from the earlier model)
% biped=matlabFunction(simplify(eval([qdot;qddot])),'Vars',{t,[q;qdot]});

% Define or modify ground_impact_acrobot according to your model specifics
% Ensure that the function signature matches with the event detection requirements of ode45
% Creation of the ground impact events function
ground_impact=matlabFunction(eval(px(2)),1,-1,...
    'File','ground_impact_acrobot','Vars',{t,[q;qdot]},...
    'Outputs',{'value', 'isterminal','direction'});

% Initial conditions for integration
q0=[pi/3;pi/4;pi-pi/6;-pi/3;-pi/6]; 
qdot0=zeros(5,1);

% Simulation of pinned acrobot
ops= odeset('reltol',1e-8,'abstol',1e-8,'Events',@ground_impact); 
dt=1/60; % 60 fps; time increment in simulations and animations

fprintf('\n Simulating...\n')

[t1,x1,te1,xe1]=ode45(@(t,x) biped(t,x,data),0:5e-2:10,[q0;qdot0],ops);

% Check if Deltafun is defined and modify accordingly
% Application of the impact map
impact_state=xe1';
post_impact_state=Deltafun(impact_state);

% Simulation of pinned acrobot
[t2,x2,te2,xe2]=ode45(biped,[te1:dt:te1+2],post_impact_state,ops);

fprintf('\n Setting up animation...\n')

% Add animation code here


%% Animation of the simulation results
Axis=[-1 4 0 2];
axis equal;
Time=text(1,1.8,['time= ',num2str(t1(1)),' secs']);
axis(Axis);
q=q0;

q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);
xdata=0;
ydata=0;
l=[l1 l2 l2 l1 l3];
Q=[q1 q1+q2 q1+q2+q3 q1+q2+q3+q4 q1+q2+q5];

for j=1:4
    xdata=[xdata xdata(end)+l(j)*cos(Q(j))];
    ydata=[ydata ydata(end)+l(j)*sin(Q(j))];
end

xdata=[xdata xdata(3)+l(5)*cos(Q(5))];
ydata=[ydata ydata(3)+l(5)*sin(Q(5))];

link1=line([xdata(1) xdata(2)],[ydata(1) ydata(2)],'color','red','linewidth',2);
link2=line([xdata(2) xdata(3)],[ydata(2) ydata(3)],'color','red','linewidth',2);
link3=line([xdata(3) xdata(4)],[ydata(3) ydata(4)],'linewidth',2);
link4=line([xdata(4) xdata(5)],[ydata(4) ydata(5)],'linewidth',2);
link5=line([xdata(3) xdata(6)],[ydata(3) ydata(6)],'linewidth',2);

fprintf('\n Animation is ready...\n')
ref=0; % This variable keeps track of the position of the stance foot accross multiple steps

animation_slowdown_factor=4; % >1 means slow down
for k=2:length(t1)
    t0=clock;
    drawnow;
    q=x1(k,1:5)';
    q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);
    Q=[q1 q1+q2 q1+q2+q3 q1+q2+q3+q4 q1+q2+q5];
    xdata=ref;
    ydata=0;
    for j=1:4
        xdata=[xdata xdata(end)+l(j)*cos(Q(j))];
        ydata=[ydata ydata(end)+l(j)*sin(Q(j))];
    end
    xdata=[xdata xdata(3)+l(5)*cos(Q(5))];
    ydata=[ydata ydata(3)+l(5)*sin(Q(5))];
    set(link1,'xdata',[xdata(1) xdata(2)],'ydata',[ydata(1) ydata(2)]);
    set(link2,'xdata',[xdata(2) xdata(3)],'ydata',[ydata(2) ydata(3)]);
    set(link3,'xdata',[xdata(3) xdata(4)],'ydata',[ydata(3) ydata(4)]);
    set(link4,'xdata',[xdata(4) xdata(5)],'ydata',[ydata(4) ydata(5)]);
    set(link5,'xdata',[xdata(3) xdata(6)],'ydata',[ydata(3) ydata(6)]);
    set(Time,'String',['time= ',num2str(round(t1(k),1)),' secs']);
    axis(Axis);
    while etime(clock,t0)<animation_slowdown_factor*(t1(k)-t1(k-1))
    end
end


ref=xdata(5);

for k=2:length(t2)
    t0=clock;
    drawnow;
    q=x2(k,1:5)';
    q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);
    Q=[q1 q1+q2 q1+q2+q3 q1+q2+q3+q4 q1+q2+q5];
    xdata=ref;
    ydata=0;
    for j=1:4
        xdata=[xdata xdata(end)+l(j)*cos(Q(j))];
        ydata=[ydata ydata(end)+l(j)*sin(Q(j))];
    end
    xdata=[xdata xdata(3)+l(5)*cos(Q(5))];
    ydata=[ydata ydata(3)+l(5)*sin(Q(5))];
    set(link1,'xdata',[xdata(1) xdata(2)],'ydata',[ydata(1) ydata(2)]);
    set(link2,'xdata',[xdata(2) xdata(3)],'ydata',[ydata(2) ydata(3)]);
    set(link3,'xdata',[xdata(3) xdata(4)],'ydata',[ydata(3) ydata(4)]);
    set(link4,'xdata',[xdata(4) xdata(5)],'ydata',[ydata(4) ydata(5)]);
    set(link5,'xdata',[xdata(3) xdata(6)],'ydata',[ydata(3) ydata(6)]);
    set(Time,'String',['time= ',num2str(round(t2(k),1)),' secs']);
    axis(Axis);
    while etime(clock,t0)<animation_slowdown_factor*(t2(k)-t2(k-1))
    end
end