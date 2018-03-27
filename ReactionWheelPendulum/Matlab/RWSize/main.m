%% Constant parameters
g = 9.81; % gravity
k = 0.0369; % the motor torque constant
R = 0.608; % resistance
Umax = 24; % max. voltage

lp = 0.35; % l of the pendulum
mp = 0.350; % mass of the pendulum
Jp = 0.005; % the penudulm's inertia

lr = 0.5; % l of the wheel

Cp=0; % friction
Cr=0; % friction
%% array of r values
rArray=0:0.001:0.2;

%% get the output
MODEL_TYPE = 0; % 1 for linear and 0 for nonliner

angleArray=zeros(size(rArray));
for i=1:length(rArray)
    
    %choose r 
    r=rArray(i);
    
    % First we comoute the wheel-dependednt parameters
    v = 0.020*pi*r^2; % volume
    mr = v*2698.9; % mass of the wheel
    Jr = 0.0000181 + mr*r*r/2; % wheel's inertia
    J = Jp + mp*lp*lp + mr*lr*lr; % 
    ml = mp*lp + mr*lr; 

    % construct the array of parameters
    params = [J, Jr, Cp, Cr, R, k, Umax, ml*g]; 

    % get the ode function handle. 
    odefun = GetHandleRWode(params,MODEL_TYPE);

    % get the Flag function handle
    flagfun = @(x0) CheckIfCanTouchZero(odefun,x0);

    % compute the max angle using bisection
    angleArray(i) = Bisection(flagfun,0,1,0.0001);
end

%% Display the output
plot(2*rArray,angleArray);grid;

