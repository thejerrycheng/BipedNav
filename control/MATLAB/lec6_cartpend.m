%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ECE1658
%% Lecture 6
%% Virtual constraint for the cart-pendulum
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearvars
close all
clc
syms M m l g q1 q2 q1dot q2dot tau t real

q=[q1;q2];
qdot=[q1dot;q2dot];
rc1=[q1;0];
rc2=rc1+l*[cos(q2);sin(q2)];
rc1dot=jacobian(rc1,q)*qdot;
rc2dot=jacobian(rc2,q)*qdot;

K=simplify(1/2*M*rc1dot'*rc1dot+1/2*m*rc2dot'*rc2dot)
P=m*g*rc2(2)
B=[1;0] % Input matrix
% Extract the square symmetric matrix of the kinetic energy
D=simplify(hessian(K,qdot))

% Computation of matrix C(q,qdot)
C = sym(zeros(2,2));
for i=1:2
    for j=1:2
        for k=1:2
            C(k,j)=C(k,j)+1/2*(diff(D(k,j),q(i))+diff(D(k,i),q(j))-diff(D(i,j),q(k)))*qdot(i);
        end
    end
end
C
% Computation of gradient of the potential

G = jacobian(P,q)'

syms L K1 K2 real
%h=q1+L*cos(q2);
%h=l*sin(q2)-l
h=q1+L*cos(q2);
A=simplify(jacobian(h,q)*inv(D)*B)


% Computation of qddot
% D qddot + C*qdot + G = B tau
% qddot = simplify(inv(D)*(-C*qdot-G+B*tau));
qddot = simplify(D\(-C*qdot-G+B*tau));
y=h;
ydot=jacobian(y,[q;qdot])*[qdot;qddot];
yddot=jacobian(ydot,[q;qdot])*[qdot;qddot];
Tau=simplify(solve(yddot+K1*y+K2*ydot,tau));

Qddot=simplify(subs(qddot,Tau));

%% Physical parameters
l=1;
L=0.5;
m=1;
M=1;
g=9.81;
lambda=3;
K1=lambda^2;
K2=2*lambda;

f=subs(subs([qdot;Qddot]));
cartpend=matlabFunction(f,'Vars',{t,[q;qdot]});

ops= odeset('reltol',1e-8,'abstol',1e-8);
dt=1/60; % 60 fps; time increment in simulations and animations

% Initial condition
q0=[L-2;0];
qdot0=[0;0];
% qdot0=[0;-5];

[t,x]=ode45(cartpend,0:dt:15,[q0;qdot0],ops);
q=x(:,1:2);
q1=q(:,1);
q2=q(:,2);
% Animation of the simulation results
Axis=[-2 2 -2 2];
axis equal;
Time=text(1,1.8,['time= ',num2str(t(1)),' secs']);
axis(Axis);
hold on;
Q1=q1(1);
Q2=q1(2);
line([Axis(1) Axis(2)],[0 0],'color','k')
line([0 0],[Axis(3) Axis(4)],'color','r')
cart=plot(Q1,0,'s','Markersize',30);
rod=line([Q1 Q1+l*cos(Q2)],[0 l*sin(Q2)]);
midpoint=plot(Q1+L*cos(Q2),L*sin(Q2),'.','Markersize',15);
pendulum=plot(Q1+l*cos(Q2),l*sin(Q2),'.','Markersize',10);

fprintf('\n Animation is ready...\n')


animation_slowdown_factor=1; % >1 means slow down
for k=2:length(t)
    t0=clock;
    drawnow;
    Q1=q1(k);
    Q2=q2(k);
    set(cart,'xdata',Q1);
    set(rod,'xdata',[Q1 Q1+l*cos(Q2)],'ydata',[0 l*sin(Q2)]);
    set(pendulum,'xdata',Q1+l*cos(Q2),'ydata',l*sin(Q2));
    set(midpoint,'xdata',Q1+L*cos(Q2),'ydata',L*sin(Q2));
    set(Time,'String',['time= ',num2str(round(t(k),1)),' secs']);
    axis(Axis);
    while etime(clock,t0)<animation_slowdown_factor*(t(k)-t(k-1))
    end
end
