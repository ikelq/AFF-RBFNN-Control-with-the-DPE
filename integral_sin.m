
clear
clc
close all
% y(1)=2;
% t=1:0.01:50;
% n=length(t);
% size=0.01;
% 
% for i=2:n
%     dy=-sin((i-1)*size);
%     y(i)=y(i-1)+size*dy;
% end
% plot(y)


tspan=[0,20];
y0=0;
[tt,yy]=ode45(@vdy,tspan,y0);
plot(yy)

function dy = vdy(t, y)
dy = -sin(t);
end

