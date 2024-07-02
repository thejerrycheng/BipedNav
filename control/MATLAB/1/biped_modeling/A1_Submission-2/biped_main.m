%% ECE1658: Assignment 1 

close all
clear all
clc

%% 2) Symbolic Computations

% 2.1) Define numerical variables
l1 = 0.5; 
l2 = 0.5; 
l3 = 0.3; % lengths

m1 = 0.05; 
m2 = 0.5; 
m3 = 0.3; 
m4 = m2; 
m5 = m1; % masses
m6 = 0.5;

g = 9.81; % gravitational acceleration

% 2.2) Define symbolic variables
fprintf('\n Initializing symbolic variables...\n')

syms t q1 q2 q3 q4 q5 x1 x2 q1dot q2dot q3dot q4dot q5dot x1dot x2dot real
syms tau1 tau2 tau3 tau4 real

q = [q1; q2; q3; q4; q5];
qbar = [q; x1; x2];

qdot = [q1dot; q2dot; q3dot; q4dot; q5dot];
qbardot = [qdot; x1dot; x2dot];

tau = [tau1; tau2; tau3; tau4];

% 2.3)
fprintf('\n Symbolic model computation...\n')

% 2.3.a) Define position of masses
r1 = [x1; x2];%
r2 = [x1; x2] + ...
    l1 * [cos(q1); sin(q1)];%
r3 = [x1; x2] + ...
    l1 * [cos(q1); sin(q1)] + ...
    l2 * [cos(q1+q2); sin(q1+q2)];%
r4 = [x1; x2] + ...
    l1 * [cos(q1); sin(q1)] + ...
    l2 * [cos(q1+q2) + cos(q1+q2+q3); sin(q1+q2) + sin(q1+q2+q3)];%
r5 = [x1; x2] + ...
    l1 * [cos(q1) + cos(q1+q2+q3+q4); sin(q1) + sin(q1+q2+q3+q4)] + ...
    l2 * [cos(q1+q2) + cos(q1+q2+q3); sin(q1+q2) + sin(q1+q2+q3)];%
r6 = [x1; x2] + ...
    l1 * [cos(q1); sin(q1)] + ...
    l2 * [cos(q1+q2); sin(q1+q2)] + ...
    l3 * [cos(q1+q2+q5); sin(q1+q2+q5)];%

% 2.3.b) Compute time derivatives of centres of mass
r1dot = jacobian(r1,qbar) * qbardot;
r2dot = jacobian(r2,qbar) * qbardot;
r3dot = jacobian(r3,qbar) * qbardot;
r4dot = jacobian(r4,qbar) * qbardot;
r5dot = jacobian(r5,qbar) * qbardot;
r6dot = jacobian(r6,qbar) * qbardot;

% 2.3.c) Define the total kinetic energy of the robot
K= (1/2) * (...
    m1 * (r1dot' * r1dot) + ...
    m2 * (r2dot' * r2dot) + ...
    m3 * (r3dot' * r3dot) + ...
    m4 * (r4dot' * r4dot) + ...
    m5 * (r5dot' * r5dot) + ...
    m6 * (r6dot' * r6dot) ...
    );

% 2.3.d) Extract the square symmetric matrix of the kinetic energy
Dbar = simplify(hessian(K,qbardot));

% 2.3.e) Extract the matrix of the kinetic energy of the pinned robot
D = Dbar(1:5,1:5);

% 2.4) Define the potential energy of the pinnedrobot + compute gradient
P = g * [ 0 1 ] * ( ...
    m1 * r1 + ...
    m2 * r2 + ...
    m3 * r3 + ...
    m4 * r4 + ...
    m5 * r5 + ...
    m6 * r6 ...
    );

G = jacobian(P,q)';

% 2.5) Input matrix of pinned robot
B = [0 0 0 0;    % Corresponds to q1, which is unactuated
     1 0 0 0;    % Corresponds to q2
     0 1 0 0;    % Corresponds to q3
     0 0 1 0;    % Corresponds to q4
     0 0 0 1];   % Corresponds to q5

% 2.6) Computation of matrix C(q,qdot) of pinned robot
C = sym(zeros(5,5));

% Iterate over indices of the matrix
for k = 1:5
    for j = 1:5
        % Sum over Cristoffel symbols
        for i = 1:5
            C(k,j) = C(k,j) + (1/2) * (...
                diff(D(k,j),q(i)) + diff(D(k,i),q(j)) - diff(D(i,j),q(k)) ...
                ) * qdot(i);
        end
    end
end

% 2.7) Pre-compute values needed for the impact map
px = r5 - r1; % position vector px for the foot experiencing impact
E = [ jacobian(px, q) eye(2) ]; % Jacobian of p = px + x

% 2.8) Create functions from the symbolic expressions
Dfun = matlabFunction(D, 'Vars', { q });
Dbarfun = matlabFunction(Dbar, 'Vars', { q });
pxfun = matlabFunction(px, 'Vars', { q });
Efun = matlabFunction(E, 'Vars', { q });
Cfun = matlabFunction(C, 'Vars', { q, qdot });
Gfun = matlabFunction(G, 'Vars', { q });

% 2.9) Create the structure array
fprintf('\n Cache objects in data structure... \n ')

data.D = Dfun;
data.Dbar = Dbarfun;
data.E = Efun;
data.C = Cfun;
data.G = Gfun;
data.B = B;

%% 3) ODE Function and Control Design

fprintf('\n Defining dynamics ... \n ')

% 3.1) Define quantities for control

% Define numerical variables for reference joint angles
q2ref = pi/6; 
q3ref = pi + pi/6; 
q4ref = -pi/6; 
q5ref = -pi/6;

% Place them in a column vector qref
qref = [q2ref; q3ref; q4ref; q5ref];

% Determine control gains Kp and Kd (poles at -20, -20)
K = acker([0 1 ; 0 0], [0 ; 1], [-20 -20]);
Kp = K(1); Kd = K(2);

% Define the 4x5 matrix H
H = [ zeros(4, 1) eye(4) ];

% Add variables to the data structure
data.qref = qref;
data.Kp = Kp;
data.Kd = Kd;
data.H = H;

% 3.2) Verify vector relative degree condition

% Define variables
syms theta real

B_perp = [1 0 0 0 0];

sigma = [theta; q2ref; q3ref; q4ref; q5ref];
sigma_prime = diff(sigma, theta); % this is essentially [1; 0; 0; 0; 0] 

D_sigma = subs(D, [q1; q2; q3; q4; q5], sigma);

% Compute desired value
expression = B_perp * D_sigma * sigma_prime;
value = double(simplify(expression)); % double(.) since indep. of theta

% Verify not zero
if ~isempty(nonzeros(value))
    fprintf('Vector relative degree condition satisfied with value:')
    disp(value)
else
    fprintf('Vector relative degree condition not satisfied')
end

% 3.3) See biped.m

%% 4) The Impact Map

% 4.1) See impact_map.m

%% 5) Setting up simulations

fprintf('\n Setting up simulations...\n')

% 5.1) Creation of the ground impact events function
ground_impact = matlabFunction(...
    px(2), 1, -1, 'File','ground_impact', ...
    'Vars', { t, [q ; qdot] }, ...
    'Outputs', { 'value', 'isterminal', 'direction' } ...
    );

% 5.2) Integration options
ops = odeset('RelTol', 1e-8, 'AbsTol', 1e-8, 'Events', @ground_impact); 

% 5.3) Initial conditions for integration
q0 = [pi/3; pi/4; pi-pi/6; -pi/3; -pi/6]; 
qdot0 = zeros(5,1);

% 5.4) Simulate robot until first impact

fprintf('\n Simulating...\n')

[t1, x1, te1, xe1] = ode45(@(t,x) biped(t,x,data), 0:5e-2:10, [q0; qdot0], ops);

% 5.5) Compute post-impact state
post_impact_state = impact_map(xe1',data);

% 5.6) Simulate robot until second impact
[t2, x2] = ode45(@(t,x) biped(t,x,data), te1 + 0:5e-2:10, post_impact_state, ops);

%% 5.7) Animation of the simulation results

clf
fprintf('\n Setting up animation...\n')

% Copied from animation_snippet.txt
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

% Create a VideoWriter object to save the animation
videoFilename = 'animation.avi';
video = VideoWriter(videoFilename);
video.FrameRate = 1 / (mean(diff(t1)) * animation_slowdown_factor); % Adjust frame rate based on time steps
open(video);
pause;

for k=2:length(t1)
    t0=clock;
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
    drawnow;
    
    % Capture the plot as a frame and write it to the video
    frame = getframe(gcf);
    writeVideo(video, frame);
    
    while etime(clock,t0)<animation_slowdown_factor*(t1(k)-t1(k-1))
    end
end

ref=xdata(5);

for k=2:length(t2)
    t0=clock;
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
    drawnow;
    
    % Capture the plot as a frame and write it to the video
    frame = getframe(gcf);
    writeVideo(video, frame);
    
    while etime(clock,t0)<animation_slowdown_factor*(t2(k)-t2(k-1))
    end
end

% Close the video file
close(video);


% fprintf('\n Animation is ready...\n')
% ref=0; % This variable keeps track of the position of the stance foot accross multiple steps
% 
% animation_slowdown_factor=4; % >1 means slow down
% for k=2:length(t1)
%     t0=clock;
%     drawnow;
%     q=x1(k,1:5)';
%     q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);
%     Q=[q1 q1+q2 q1+q2+q3 q1+q2+q3+q4 q1+q2+q5];
%     xdata=ref;
%     ydata=0;
%     for j=1:4
%         xdata=[xdata xdata(end)+l(j)*cos(Q(j))];
%         ydata=[ydata ydata(end)+l(j)*sin(Q(j))];
%     end
%     xdata=[xdata xdata(3)+l(5)*cos(Q(5))];
%     ydata=[ydata ydata(3)+l(5)*sin(Q(5))];
%     set(link1,'xdata',[xdata(1) xdata(2)],'ydata',[ydata(1) ydata(2)]);
%     set(link2,'xdata',[xdata(2) xdata(3)],'ydata',[ydata(2) ydata(3)]);
%     set(link3,'xdata',[xdata(3) xdata(4)],'ydata',[ydata(3) ydata(4)]);
%     set(link4,'xdata',[xdata(4) xdata(5)],'ydata',[ydata(4) ydata(5)]);
%     set(link5,'xdata',[xdata(3) xdata(6)],'ydata',[ydata(3) ydata(6)]);
%     set(Time,'String',['time= ',num2str(round(t1(k),1)),' secs']);
%     axis(Axis);
%     while etime(clock,t0)<animation_slowdown_factor*(t1(k)-t1(k-1))
%     end
% end
% 
% 
% ref=xdata(5);
% 
% for k=2:length(t2)
%     t0=clock;
%     drawnow;
%     q=x2(k,1:5)';
%     q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);
%     Q=[q1 q1+q2 q1+q2+q3 q1+q2+q3+q4 q1+q2+q5];
%     xdata=ref;
%     ydata=0;
%     for j=1:4
%         xdata=[xdata xdata(end)+l(j)*cos(Q(j))];
%         ydata=[ydata ydata(end)+l(j)*sin(Q(j))];
%     end
%     xdata=[xdata xdata(3)+l(5)*cos(Q(5))];
%     ydata=[ydata ydata(3)+l(5)*sin(Q(5))];
%     set(link1,'xdata',[xdata(1) xdata(2)],'ydata',[ydata(1) ydata(2)]);
%     set(link2,'xdata',[xdata(2) xdata(3)],'ydata',[ydata(2) ydata(3)]);
%     set(link3,'xdata',[xdata(3) xdata(4)],'ydata',[ydata(3) ydata(4)]);
%     set(link4,'xdata',[xdata(4) xdata(5)],'ydata',[ydata(4) ydata(5)]);
%     set(link5,'xdata',[xdata(3) xdata(6)],'ydata',[ydata(3) ydata(6)]);
%     set(Time,'String',['time= ',num2str(round(t2(k),1)),' secs']);
%     axis(Axis);
%     while etime(clock,t0)<animation_slowdown_factor*(t2(k)-t2(k-1))
%     end
% end
