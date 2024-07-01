%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ECE1658
%% Lecture 4
%% Symbolic model of the walking acrobot robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear all
clc
% Define symbolic variables
fprintf('\n Initializing symbolic variables...\n')
syms l lc Iz m g real
syms t q1 q2 x1 x2 q1dot q2dot x1dot x2dot tau real

q=[q1;q2];x=[x1;x2]; qbar=[q;x];
qdot=[q1dot;q2dot];  xdot=[x1dot;x2dot]; qbardot=[qdot;xdot];

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

% Input matrix of pinned robot
B=[0;1];

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

% Computation of qddot (pinned robot)

qddot = simplify(inv(D)*(-C*qdot-G+B*tau));

fprintf('\n Impact map computation...\n')

%% Physical parameters
l=1;
lc=0.5;
m=1;
Iz=1/12*m*l^2;
g=9.81;
tau=0;

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


fprintf('\n Setting up simulations...\n')

%% Setting up simulations

% ODE of pinned acrobot
acrobot=matlabFunction(simplify(eval([qdot;qddot])),'Vars',{t,[q;qdot]});

% Creation of the ground impact events function
ground_impact_acrobot=matlabFunction(eval(px(2)),1,-1,...
    'File','ground_impact_acrobot','Vars',{t,[q;qdot]},...
    'Outputs',{'value', 'isterminal','direction'});

% Initial conditions for integration
q0=[pi/2-pi/6;-pi/6];
qdot0=[2;-10];

% Simulation of pinned acrobot
ops= odeset('reltol',1e-8,'abstol',1e-8,'Events',@ground_impact_acrobot); 
dt=1/60; % 60 fps; time increment in simulations and animations

fprintf('\n Simulating...\n')

[t1,x1,te1,xe1]=ode45(acrobot,0:dt:10,[q0;qdot0],ops);

% Application of the impact map
impact_state=xe1';
post_impact_state=Deltafun(impact_state);

% Simulation of pinned acrobot
[t2,x2,te2,xe2]=ode45(acrobot,[te1:dt:te1+2],post_impact_state,ops);

fprintf('\n Setting up animation...\n')

%% Animation of the simulation results
ref=0;
Axis=[-1 4 0 2];
axis equal;
Time=text(1,1.8,['time= ',num2str(t1(1)),' secs']);
axis(Axis); 
stance_leg=line([ref l*cos(q0(1))],[0 l*sin(q0(1))],'color','red','linewidth',2);
swing_leg=line([ref+l*cos(q0(1)) ref+l*cos(q0(1))+l*cos(q0(1)+q0(2))],...
    [l*sin(q0(1)) l*sin(q0(1))+l*sin(q0(1)+q0(2))],'linewidth',2);

fprintf('\n Animation is ready...\n')

pause
animation_slowdown_factor=1; % >1 means slow down
for k=2:length(t1)
    t0=clock;
    drawnow;
    q=x1(k,1:2)';
    xdata1=[ref ref+l*cos(q(1))];
    xdata2=[ref+l*cos(q(1)) ref+l*cos(q(1))+l*cos(q(1)+q(2))];
    ydata1=[0 l*sin(q(1))];
    ydata2=[l*sin(q(1)) l*sin(q(1))+l*sin(q(1)+q(2))];
    set(stance_leg,'xdata',xdata1,'ydata',ydata1);
    set(swing_leg,'xdata',xdata2,'ydata',ydata2);
    set(Time,'String',['time= ',num2str(round(t1(k),1)),' secs']);
    axis(Axis);
    while etime(clock,t0)<animation_slowdown_factor*(t1(k)-t1(k-1))
    end
end

ref=ref+l*(cos(xe1(1))+cos(xe1(1)+xe1(2)));

for k=2:length(t2)
    t0=clock;
    drawnow;
    q=x2(k,1:2)';
    xdata1=[ref ref+l*cos(q(1))];
    xdata2=[ref+l*cos(q(1)) ref+l*cos(q(1))+l*cos(q(1)+q(2))];
    ydata1=[0 l*sin(q(1))];
    ydata2=[l*sin(q(1)) l*sin(q(1))+l*sin(q(1)+q(2))];
    set(stance_leg,'xdata',xdata1,'ydata',ydata1);
    set(swing_leg,'xdata',xdata2,'ydata',ydata2);
    set(Time,'String',['time= ',num2str(round(t2(k),1)),' secs']);
    axis(Axis);
    while etime(clock,t0)<animation_slowdown_factor*(t2(k)-t2(k-1))
    end
end


