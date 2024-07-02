%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ECE1658
%% Lecture 3
%% Symbolic model of the acrobot robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearvars
close all
clc
% Define symbolic variables
syms l lc Iz m g real
syms t q1 q2 q1dot q2dot tau real

%% Physical parameters
l=1;
lc=0.5;
Iz=1/12*m*l^2;
m=1;
g=9.81;
q0=[pi/4;pi/4];
qdot0=[0;0];
tau=0;

q=[q1;q2];
qdot=[q1dot;q2dot];

% Define centres of mass of two links
rc1=lc*[cos(q1);sin(q1)];
rc2=l*[cos(q1);sin(q1)]+lc*[cos(q1+q2);sin(q1+q2)];

% Compute time derivatives of centres of mass
rc1dot=jacobian(rc1,q)*qdot;
rc2dot=jacobian(rc2,q)*qdot;

% Define the total kinetic energy of the robot
K=1/2*m*(rc1dot'*rc1dot+rc2dot'*rc2dot)+1/2*Iz*(q1dot^2+(q1dot+q2dot)^2);
K=simplify(K);

% Extract the square symmetric matrix of the kinetic energy
D=simplify(hessian(K,qdot));

% Define the potential energy of the robot

P = m*g*(lc*sin(q1)+l*sin(q1)+lc*sin(q1+q2));

% Input matrix

B=[0;1];

% Computation of matrix C(q,qdot)
C = sym(zeros(2,2));
for i=1:2
    for j=1:2
        for k=1:2
            C(k,j)=C(k,j)+1/2*(diff(D(k,j),q(i))+diff(D(k,i),q(j))-diff(D(i,j),q(k)))*qdot(i);
        end
    end
end

% Computation of gradient of the potential

G = jacobian(P,q)';

% Computation of qddot
% D qddot + C*qdot + G = B tau
% qddot = simplify(inv(D)*(-C*qdot-G+B*tau));
qddot = simplify(D\(-C*qdot-G+B*tau));

%% Physical parameters
l=1;
lc=0.5;
Iz=1/12*m*l^2;
m=1;
g=9.81;
q0=[pi/4;pi/4];
qdot0=[0;0];
tau=0;

f=subs(subs([qdot;qddot]));
acrobot=matlabFunction(f,'Vars',{t,[q;qdot]});

ops= odeset('reltol',1e-8,'abstol',1e-8);
dt=1/60; % 60 fps; time increment in simulations and animations

[t,x]=ode45(acrobot,0:dt:15,[q0;qdot0],ops);

% Animation of the simulation results
Axis=[-2 2 -2 2];
axis equal;
Time=text(1,1.8,['time= ',num2str(t(1)),' secs']);
axis(Axis); 
stance_leg=line([l*cos(q0(1))],[0 l*sin(q0(1))],'color','red','linewidth',2);
swing_leg=line([l*cos(q0(1)) l*cos(q0(1))+l*cos(q0(1)+q0(2))],...
    [l*sin(q0(1)) l*sin(q0(1))+l*sin(q0(1)+q0(2))],'linewidth',2);

fprintf('\n Animation is ready...\n')
% Create a VideoWriter object for an MP4 file
v = VideoWriter('your_animation.mp4', 'MPEG-4');

% Open the file for writing
open(v);

animation_slowdown_factor = 1; % >1 means slow down
for k = 2:length(t)
    t0 = clock;
    drawnow;
    
    % ... [your existing animation code] ...
    t0=clock;
    drawnow;
    q=x(k,1:2)';
    xdata1=[0 l*cos(q(1))];
    xdata2=[l*cos(q(1)) l*cos(q(1))+l*cos(q(1)+q(2))];
    ydata1=[0 l*sin(q(1))];
    ydata2=[l*sin(q(1)) l*sin(q(1))+l*sin(q(1)+q(2))];
    set(stance_leg,'xdata',xdata1,'ydata',ydata1);
    set(swing_leg,'xdata',xdata2,'ydata',ydata2);
    set(Time,'String',['time= ',num2str(round(t(k),1)),' secs']);
    axis(Axis);
    while etime(clock,t0)<animation_slowdown_factor*(t(k)-t(k-1))
    end

    % Capture the plot as a frame
    frame = getframe(gcf);
    
    % Write the frame to the video
    writeVideo(v, frame);

    % Ensure the animation runs at the correct speed
    while etime(clock, t0) < animation_slowdown_factor * (t(k) - t(k-1))
    end
end

% Close the video file
close(v);
% 
% animation_slowdown_factor=1; % >1 means slow down
% for k=2:length(t)
%     t0=clock;
%     drawnow;
%     q=x(k,1:2)';
%     xdata1=[0 l*cos(q(1))];
%     xdata2=[l*cos(q(1)) l*cos(q(1))+l*cos(q(1)+q(2))];
%     ydata1=[0 l*sin(q(1))];
%     ydata2=[l*sin(q(1)) l*sin(q(1))+l*sin(q(1)+q(2))];
%     set(stance_leg,'xdata',xdata1,'ydata',ydata1);
%     set(swing_leg,'xdata',xdata2,'ydata',ydata2);
%     set(Time,'String',['time= ',num2str(round(t(k),1)),' secs']);
%     axis(Axis);
%     while etime(clock,t0)<animation_slowdown_factor*(t(k)-t(k-1))
%     end
% end

% ops= odeset('reltol',1e-8,'abstol',1e-8);
% 
% [t,x]=ode45(acrobot,[0 10],[q0;qdot0],ops);
% q=x(:,1:2);
% plot(q(:,1),q(:,2));
