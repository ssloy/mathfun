function flag = CheckIfCanTouchZero(odefun, theta0)
%CheckIfCanTouchZero this function check if we can touc zero with the given
%initial condition in 1 second

x0=[0; -theta0; 0; 0];
tspan = 0:0.01:1;
[~,x] = ode45(odefun,tspan,x0);

x=x(:,2); % theta

if any(x>eps)
    flag=1;
else
    flag=-1;
end
end

