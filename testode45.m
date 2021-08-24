arg1 = 2;
arg2 = 1;
[T,Y] = ode45(@vdp1000,[0 100],[0 0], [], arg1, arg2);
plot(T,Y(:,1));


% tspan = 0:0.1:20;
% y = ode5(@vdp1,tspan,[2 0]);
% plot(tspan,y(:,1));

function dy = vdp1000(t, y, flag, arg1, arg2)
dy = zeros(2,1);    % a column vector
dy(1) = y(2);
dy(2) = -sin(t);
end


