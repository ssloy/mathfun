%% Constant parameters
g = 9.81; % gravity
k = 0.0369; % the motor torque constant
R = 0.608; % resistance
Umax = 24; % max. voltage
Imax= Umax/R; % approx. max current

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

% Thete sensor displacemnet
thetaDisplacement = 0.05;

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
Qlqr=0.3*diag([3 2 2]);
Klqr = lqr(Alin,Blin,Qlqr,Rlqr);

%% Differentiators
ap=0.75;
kp=poly([-7 -8]);
kp=kp(2:end);
xp0=[theta0;0];

ar=0.75;
kr=poly([-7 -8]);
kr=kr(2:end);
xr0=[thetaRW0;0];

%% Displacement observer
b1=-k/J;
a1=mlg/J;
b2=(J+Jr)*k/J/Jr;
a2=-mlg/J;

Lobs=[546;1100;-508];

% This part was used for LMI solver and is not actual at the moment
% AObs1=[0 1 0; 0 0 -a1; 0 0 0];
% AObs2=[0 1 0; 0 0 -0.05*a1; 0 0 0];
% CObs=[1 0 1];

% LObs=sdpvar(1,3);
% PObs=sdpvar(3);
% Q1=AObs1'*PObs+PObs*AObs1-CObs'*LObs-LObs'*CObs+3*PObs;
% Q2=AObs2'*PObs+PObs*AObs2-CObs'*LObs-LObs'*CObs+1*PObs;
% sol = optimize([Q1<=0, Q2<=0, PObs>=0.01*eye(3)],[]);
% if sol.problem
%     error('LMI not solved');
% end
% LObs=(value(LObs)/value(PObs))';

