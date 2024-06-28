%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ECE1658
%% FINAL PROJECT
%% VHC for walking acrobot
%% PARTIAL CODE TO GET STUDENTS STARTED ON THE PROJECT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear all
clc

number_steps=10;
symbolic_computations=1;
%% Physical parameters
l=1;
lc=0.5;
m=1;
Iz=1/12*m*l^2;
g=9.81;
psi=deg2rad(2);
%% Control parameters
Kp=20^2;
Kd=20*2;

%% Symbolic computations
if symbolic_computations
    % Define symbolic variables
    fprintf('\n Initializing symbolic variables...\n')
    % syms l lc Iz m g real
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
    psi=deg2rad(incline_degrees);
    gvector=[cos(psi) -sin(psi);sin(psi) cos(psi)]*[0; -1];
    P=-(m*g*lc*[cos(q1);sin(q1)]+m*g*[l*cos(q1)+lc*cos(q1+q2);l*sin(q1)+lc*sin(q1+q2)])'*gvector;
    P=simplify(P);
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
    
    %     qddot = simplify(inv(D)*(-C*qdot-G+B*tau));
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
    save('walking_acrobot_model','D','Dfun','Cfun','Gfun','Deltafun','B');
else
    fprintf('\nLoading robot model...\n')
    load('walking_acrobot_model');
end
%% HERE WRITE YOUR CODE FOR THE VHC DESIGN
% The outcome of this part should be a parameter vector a, whose components
% a_1,ldots,a_k define the polynomial phi_a(theta)=a_1 + a_2 theta + ... +
% a_k theta^(k-1)

fprintf('\nDetermining vhc...\n')


%% HERE WE DEFINE THE FUNCTION phi_a AND ITS DERIVATIVES
a=flip(a);
phi=@(theta) polyval(a,theta);
phiprime=@(theta) polyval(polyder(a),theta);
phipprime=@(theta) polyval(polyder(polyder(a)),theta);

% Using phi and its derivatives, below you should define functions sigma,
% sigmaprime, sigmapprime.

% This is the data structure to be passed to various functions
% You will need to add extra information to it.
data.Kp=Kp;
data.Kd=Kd;
data.D=Dfun;
data.C=Cfun;
data.G=Gfun;
data.B=B;
data.phi=phi;
data.phiprime=phiprime;
data.phipprime=phipprime;
data.sigma=sigma;
data.sigmaprime=sigmaprime;
data.sigmapprime=sigmapprime;

%% HERE WRITE CODE TO TEST WHETHER YOUR VHC WORKS

%% NOW YOU CAN SIMULATE THE ROBOT. PLACE YOUR CONTROLLER INSIDE THE FUNCTION acrobot AT THE END OF THIS SCRIPT
ops= odeset('reltol',1e-7,'abstol',1e-7,'Events',@ground_impact);
dt=1/60; % 60 fps; time increment in simulations and animations

fprintf('\n Simulating...\n')
%% DEFINE THE INITIAL CONDITION [q0;qdot0];

T=[];
X=[];
Te=[];
Ie=[];
Xe=[];
post_impact_state=[q0;qdot0];
% Simulate number_steps steps
for step=1:number_steps
    fprintf('\n...step %d...\n',step);
    [t,x,te,xe,ie]=ode45(@(t,x) acrobot(t,x,data),0:dt:10,post_impact_state,ops);
    % Application of the impact map
    impact_state=xe(end,:)';
    post_impact_state=Deltafun(impact_state);
    T{step}=t;
    X{step}=x;
    Ie{step}=ie;
    Te{step}=te;
    Xe{step}=xe;
end

fprintf('\n Setting up animation...\n')
figure(1);

%% Animation of the simulation results
ref=0;time_passed=0;step=1;
Axis=[-1 4 0 2];
Time=text(-1+2,1.8,['time= ','0',' secs,',' step= ',num2str(step)]);
axis(Axis);
stance_leg=line([ref l*cos(q0(1))],[0 l*sin(q0(1))],'color','red','linewidth',2);
swing_leg=line([ref+l*cos(q0(1)) ref+l*cos(q0(1))+l*cos(q0(1)+q0(2))],...
    [l*sin(q0(1)) l*sin(q0(1))+l*sin(q0(1)+q0(2))],'linewidth',2);
fprintf('\n Animation is ready...\n')

animation_slowdown_factor=1; % >1 means slow down
for step=1:length(Ie)
    t=T{step};
    x=X{step};
    xe=Xe{step};
    xe=xe(end,:);
    for k=2:length(t)
        t0=clock;
        drawnow;
        q=x(k,1:2)';
        xdata1=[ref ref+l*cos(q(1))];
        xdata2=[ref+l*cos(q(1)) ref+l*cos(q(1))+l*cos(q(1)+q(2))];
        ydata1=[0 l*sin(q(1))];
        ydata2=[l*sin(q(1)) l*sin(q(1))+l*sin(q(1)+q(2))];
        set(stance_leg,'xdata',xdata1,'ydata',ydata1);
        set(swing_leg,'xdata',xdata2,'ydata',ydata2);
        set(Time,'String',['time= ',num2str(round(time_passed+t(k),1)),' secs,',' step= ',num2str(step)]);
        current_axis=gca;
        if ref>.95*current_axis.XLim(end)
            current_axis.XLim=[.95*ref .95*ref+5];
            Time.Position=[.95*ref+2 1.8 0];
            Axis=axis;
        else
            axis(Axis)
        end
        while etime(clock,t0)<animation_slowdown_factor*(t(k)-t(k-1))
        end
    end
    time_passed=time_passed+t(end);
    ref=ref+l*(cos(xe(1))+cos(xe(1)+xe(2)));
end


%% FUNCTIONS
function xdot=acrobot(t,x,data)
q1=x(1);
q2=x(2);
q1dot=x(3);
q2dot=x(4);
q=x(1:2);
qdot=x(3:4);
Kp=data.Kp;
Kd=data.Kd;
D=data.D;
C=data.C;
G=data.G;
B=data.B;
phi=data.phi;
phiprime=data.phiprime;
phipprime=data.phipprime;
% DEFINE YOUR CONTROLLER HERE
% tau=....
qddot=inv(D(q))*(-C(x)*qdot-G(q)+B*tau);
xdot=[qdot;qddot];
end

function [value,isterminal,direction]=ground_impact(t,x)
q1=x(1);
q2=x(2);
% impact occurs when q2 = -2*q1+2*pi
value=q2+2*q1-2*pi;

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
