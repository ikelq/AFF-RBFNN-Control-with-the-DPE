clear
clc
close all

global belta p1 p2 p3 q w K1 K2 i zz1

belta=0.7;
p1=0.4;
p2=-1.1;
p3=1.0;
q=1.498;
w=1.8;

K1=3;
K2=10;

i=1;
zz1(1)=0;

tspan=[0,500];    
%tspan=[0:0.01:500];    

y0=[0.2;0.3;0.2;0.3];        

% options
% odeget(options,'Reltol')
% options=odeset(options,'Reltol',1e-6)
% odeget(options,'Reltol')
options = odeset('MaxStep', 1e-10, 'RelTol',1e-10,'AbsTol',1e-10);
[tt,yy]=ode45(@DyDt,tspan,y0);
plot(tt,yy(:,1)),title('x(t)')
figure
e=yy(:,3)-yy(:,1);
plot(tt,e)
function ydot=DyDt(t,y)
global  belta p1 p2 p3 q w K1 K2  i zz1

ydot1=y(2);
ydot2=q*cos(w*t);
%-p1*y(2)-p2*y(1)-p3*y(1)^3+

%ydot=[y(2); -p1*y(2)-p2*y(1)-p3*y(1)^3+q*cos(w*t)];
z1=y(3)-y(1);
zz1(i)=z1;
i=i+1;
de=y(4)-y(2);
z2=de+K1*z1;
dalpha=-K1*de+q*cos(w*t);
%-p1*y(2)-p2*y(1)-p3*y(1)^3

ydot3=y(4);
Tau=-z1-K2*z2+ y(3) - belta*(1-y(3)^2)*y(4)+dalpha;
ydot4=-y(3)+ belta*(1-y(3)^2)*y(4)+Tau;

ydot=[ydot1;ydot2;ydot3;ydot4];
end
