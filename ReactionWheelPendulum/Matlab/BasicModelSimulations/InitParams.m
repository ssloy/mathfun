%% Constant parameters
g = 9.81; % gravity
k = 0.0369; % the motor torque constant
R = 0.608; % resistance
Umax = 24; % max. voltage

lp = 0.35; % l of the pendulum
mp = 0.350; % mass of the pendulum
Jp = 0.005; % the penudulm's inertia

L = 0.003; % we do not know the value, but let us assume this one for controller design.

lr = 0.5; % l of the wheel

Cp=0; % friction
Cr=0; % friction

if USE_IDEAL_RW % For an idela RW we have
    r=0.055; % 11 cm diametr 
    v = 0.020*pi*r^2; % volume
    mr = v*2698.9; % mass of the wheel
    Jr = 0.0000181 + mr*r*r/2; % wheel's inertia
else % For a real system we have something out of the blue, to be updated
    mr=0.200; % 
    Jr = 8e-04; %
end

J = Jp + mp*lp*lp + mr*lr*lr; %
mlg = (mp*lp + mr*lr)*g; 

% construct the array of the mechanical parameters used for simulations
mechParams = [J, Jr, Cp, Cr, mlg]; 

%% Initial values
theta0=-0.1;
dTheta0=1;
thetaRW0=0;
dThetaRW0=-1;

%%  Control design
% The current loop is inside the amplifier. Let us assume that it is PI.
% The controller is given by [L*p+R]/[Tc*p] = [KP*p + KI]/p
desiredTCur = 0.003;
curLoopKP = L/desiredTCur;
curLoopKI = R/desiredTCur;


% linearized model is given by (without friction); 
% X = [theta; dTheta; dThetaRW]
Alin = [0 1 0; mlg/J 0 0; -mlg/J 0 0];
Blin = [0; -k/J; k*(J+Jr)/J/Jr];

% basic LQR
Rlqr=1;
Qlqr=0.1*diag([3 2 1]);
Klqr = lqr(Alin,Blin,Qlqr,Rlqr);

%% Differentiators
ap=0.75;
kp=poly([-3 -4]);
kp=kp(2:end);
xp0=[theta0;0];

ar=0.75;
kr=poly([-3 -4]);
kr=kr(2:end);
xr0=[thetaRW0;0];


