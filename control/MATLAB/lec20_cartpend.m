%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ECE1658
%% Lecture 21
%% Virtual constraint for the cart-pendulum (continued from Lec 20)
%% Lagrangian structure of constrained dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearvars
close all
clc
syms M m l g q1 q2 q1dot q2dot tau t real

animation = 1; % flag determines whether or not the animation runs
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
qddot = simplify(D\(-C*qdot-G+B*tau));
y=h;
ydot=jacobian(y,[q;qdot])*[qdot;qddot];
yddot=jacobian(ydot,[q;qdot])*[qdot;qddot];
Tau=simplify(solve(yddot+K1*y+K2*ydot,tau));

Qddot=simplify(subs(qddot,Tau));

%% Computation of the constrained dynamics
syms theta thetadot real

Bperp=[0 1];
sigma=[-L*cos(theta);theta];
sigmaprime=diff(sigma,theta);
sigmapprime=diff(sigmaprime,theta);
M=subs(Bperp*D*sigmaprime,q,sigma);

Psi1=simplify(-subs((Bperp*G)/M,q,sigma))
Psi2=-subs(Bperp*(D*sigmapprime+subs(C,[q;qdot],[sigma;sigmaprime])...
    *sigmaprime)/M,q,sigma);
Psi2=simplify(Psi2);
% return
%% Physical parameters
l=1;
L=0.5;
m=1;
M=1;
g=9.81;
lambda=3;
% Initial condition
q0=[L-2;0];
qdot0=[0;0];
% qdot0=[0;5];
K1=lambda^2;
K2=2*lambda;

f=subs(subs([qdot;Qddot]));
cartpend=matlabFunction(f,'Vars',{t,[q;qdot]});


dt=1/30; % 60 fps; time increment in simulations and animations
ops= odeset('reltol',1e-8,'abstol',1e-8);
[t,x]=ode45(cartpend,0:dt:10,[q0;qdot0],ops);
q=x(:,1:2);qdot=x(:,3:4);
q1=q(:,1);
q2=q(:,2);

sigmaprimefun=matlabFunction(eval(sigmaprime));

theta0=q2(end);
thetadot0=(qdot(end,:)*sigmaprimefun(theta0))/norm(sigmaprimefun(theta0))^2;


% Phase portrait of constrained dynamics
figure(1);
Psi1fun=matlabFunction(eval(vectorize(Psi1)));
Psi2fun=matlabFunction(eval(vectorize(Psi2)));

[Theta,Thetadot]=meshgrid(linspace(-pi,pi,100),linspace(-10,10,100));
hold on;
streamslice(Theta,Thetadot,Thetadot,Psi1fun(Theta)+Psi2fun(Theta).*Thetadot.^2);
constrained_dyn=matlabFunction([thetadot;eval(Psi1+Psi2*thetadot^2)],...
    'Vars',{'t',[theta;thetadot]});
[t1,x]=ode45(constrained_dyn,[0 5],[theta0;thetadot0],ops);
plot(mod(x(:,1)+pi,2*pi)-pi,x(:,2),'ro','linewidth',2);
hold off
axis tight

xlabel('theta')
ylabel('theta dot')

if animation
    figure(2);
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
    pendulum=plot(Q1+l*cos(Q2),l*sin(Q2),'.','Markersize',30);

    fprintf('\n Animation is ready...\n')
    pause

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
end