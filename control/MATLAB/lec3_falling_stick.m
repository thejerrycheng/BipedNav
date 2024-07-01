%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ECE1658
%% Lecture 3
%% Symbolic model of the falling stick
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Variable initialization
clearvars
close all
clc
% Define symbolic variables
syms l Iz m g real
syms t q1 q2 q3 q1dot q2dot q3dot real

q=[q1;q2;q3];
qdot=[q1dot;q2dot;q3dot];

%% Equations of motion
% Potential, gravity force, and D matrix
P=m*g*q3;
G=jacobian(P,q)';
D=diag([Iz m m]);
% EOMs
qddot = -inv(D)*G;
% point of contact with the ground, assuming touchdown has occurred
tip= [q2;q3]+l*[sin(q1);-cos(q1)]; % tip 1
E=jacobian(tip,q);


%% Physical parameters
m=1;
l=1;
g=9.81;
Iz=1/12*m*l^2;

%% Impact map
Delta=[1 0 0 0 0]*inv([D -E';E zeros(2,2)])*[D*qdot;0;0];
Deltafun=matlabFunction([q1;eval(Delta)],'Vars',{[q;qdot]});

%% Setting up simulations

% ODE of the unpinned stick
falling_stick=matlabFunction([qdot;eval(qddot)],'Vars',{t,[q;qdot]});

% ODE of the pinned stick (pendulum)
pendulum=matlabFunction([q1dot;eval(m*g*l*sin(q1)/(m*l^2+Iz))],...
    'Vars',{t,[q1;q1dot]});

% Creation of the ground impact events function
ground_impact_stick=matlabFunction([eval(tip(2));eval(subs(tip(2),q1,q1+pi))],...
[1;1],[-1;-1],'File','ground_impact_stick','Vars',{t,[q;qdot]},...
    'Outputs',{'value', 'isterminal','direction'});

% Initial conditions for integration
q0=[pi/3;-1;2]; % q0(3) must be greater than one to make sure the stick
               % is initially falling
q0dot=[-1;2;.5]; % makes tip 1 hit the ground
%q0dot=[2;2;.5]; % makes tip 2 hit the ground
% Integration options
ops= odeset('reltol',1e-8,'abstol',1e-8,'Events',@ground_impact_stick); 
ops2= odeset('reltol',1e-8,'abstol',1e-8);
dt=1/60; % 60 fps; time increment in simulations and animations

% Simulation of unpinned stick in free fall
% The simulation stops when either tip hits the ground
% The integer ie indicates which tip of the stick has impacted the ground
[t,x,te,xe,ie]=ode45(falling_stick,0:dt:10,[q0;q0dot],ops);

% Application of the impact map
impact_state=xe'+(ie==2)*[pi;zeros(5,1)];
post_impact_state=Deltafun(impact_state);
fprintf('\n tip number %d has impacted the ground\n',ie)

%Simulation of the pinned stick (a pendulum)
[tpend,xpend]=ode45(pendulum,[te:dt:te+2],post_impact_state,ops2);

%% Animation of the simulation results
foot_position=eval(subs(tip,q,impact_state(1:3)));
stick=line([q0(2)-l*sin(q0(1)),q0(2)+l*sin(q0(1))],...
    [q0(3)+l*cos(q0(1)) q0(3)-l*cos(q0(1))],...
    'linewidth',2);
Axis=[-3 3 -2 3];
Time=text(-.5,2.5,num2str(t(1)));

axis(Axis);
axis equal
line([-2 2],[0 0],'color','k','linewidth',2)


animation_slowdown_factor=1; % >1 means slow down
for k=2:length(t)
    t0=clock;
    drawnow;
    q=x(k,1:3)';
    xdata=[q(2)-l*sin(q(1)),q(2)+l*sin(q(1))];
    ydata=[q(3)+l*cos(q(1)),q(3)-l*cos(q(1))];
    set(stick,'xdata',xdata,'ydata',ydata);
    set(Time,'String',['time= ',num2str(round(t(k),1)),' secs']);
    axis(Axis);
    while etime(clock,t0)<animation_slowdown_factor*(t(k)-t(k-1))
    end
end

for k=2:length(tpend)
    t0=clock;
    drawnow;
    q1=xpend(k,1);%+(ie==2)*pi;
    %     q=[xpend(k,1);xe(2)-l*sin(q1);l*cos(q1)];
    %     xdata=[q(2)-l*sin(q(1)),q(2)+l*sin(q(1))];
    %     ydata=[q(3)+l*cos(q(1)),q(3)-l*cos(q(1))];
    xdata=[foot_position(1),foot_position(1)-2*l*sin(q1)];
    ydata=[foot_position(2),foot_position(2)+2*l*cos(q1)];
    set(stick,'xdata',xdata,'ydata',ydata);
    set(Time,'String',['time= ',num2str(round(tpend(k),1)),' secs']);
    axis(Axis);
    while etime(clock,t0)<animation_slowdown_factor*(tpend(k)-tpend(k-1))
    end
end
    

