%% Some clean up

clc;
clear all;
close all;

%% %%% Test case definiton %%%%%%%%%%%%%%%%%%%%%%%%%%
u0          = 5;          % Longitudinal Speed [m/s]

trackPath   = load('TrackPath_smooth.mat');
trackPath   = trackPath.TrackPath_smooth;

X0          = trackPath(1,1);
Y0          = trackPath(1,2);
X1          = trackPath(5,1);
Y1          = trackPath(5,2);
Psi0        = atan2(Y1-Y0,X1-X0);

ifStepResponse = false;
ifDLC = false;
ifSWD = false;
if ifSWD
    load yawRateReference.mat;
end
targetSpeed = 120/3.6;

%% %%%% Simulation parameters %%%%%%%%%%%%%%%%%%%%%%%
sampleTime  = .01;             % Simulation Step Size [s]
simTime     = 30;               % Simulation end time [s]

%% %%% Car parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
m       = 217.4;                % Mass [kg]
Izz     = 133.32;                  % Moment of inertia about z axis [kgm^2]
L       = 1.53;                 % Wheelbase [m]
l1      = 0.544*L;               % Distance from COG to front axle [m]
l2      = l1-L;                 % Distance from COG to rear axle [m]
l       = [l1;l1;l2;l2];
w       = [1.25;-1.25;1.2;-1.2]/2;  % Track width [m]
wRadius = 0.22;
G_ratio = 16;
m_us    = 28;
ms      = m-m_us;               % Sprung mass [kg]
Ixx     = 30.031;                  % Vehicle inertia about X axis
Iyw     = 2;
cPhi    = (4.2433e4);                  % Vehicle total roll stiffness
kPhi    = 0;                    % Vehicle total roll damping
cLambda = 0.4725;               % Vehcile roll stiffness distribution
kLambda = 0.57;                 % Vehicle roll damping distribution

h       = 0.280;                % Height of CoG
h1      = 0.0402;                % Front roll center height
h2      = 0.094;                % Rear roll center height
h0      = h - (l1*h2 - l2*h1)/L;
% Height of CoG above roll axis
hp      = h + 0.15;
A       = 1.14;                 % Frontal area
rho     = 1.18415;              % density of air
Cd      = 1.21;                 % Drag coefficient
Cl      = 2.83*0.6;             % Lift coefficient
g       = 9.81;                 % Acceleration due to gravity



%%
velocityLimit = 80/3.6;
lateralAccelerationLimit = 1*g;
accelerationLimit = 1*g;
decelerationLimit = -g;
headingErrorDependency = 0.4;


%% Cornering stiffness
Fz = m*g*[-l2;-l2;l1;l1]/(2*L);

Ca = 2e-15*(Fz).^6 -2e-11*(Fz).^5 + 5e-8*(Fz).^4 ...
    - 6e-5*(Fz).^3 + 0.0066*(Fz).^2 + 53.121*(Fz) + 2.9346;

Ku = ((Ca(3)+Ca(4))*l2-(Ca(1)+Ca(2))*l1)/((Ca(1)+Ca(2))*(Ca(3)+Ca(4))*(l1-l2));

Fz0     = 600;
mu0     = 1.5;
mu1     = -1.9567e-04;
mu      = mu0*(1-mu1*(Fz-Fz0));

%% Tire Data
load tireDataY
load tireDataX
tireLoad = tireDataY(1,2:end);
tireLoad = -tireLoad;

tireSlipY = tireDataY(2:end,1)*pi/180;
tireSlipX = tireDataX(2:end,1);
tireForceY1 = -tireDataY(2:26,2:end);
tireForceY2 = -tireForceY1;
tireForceX1 = tireDataX(2:11,2:end);
tireForceX2 = -tireForceX1;



tireForceX = cat(1,tireForceX1,flipud(tireForceX2));
tireForceY = cat(1,tireForceY1,flipud(tireForceY2))*0.5775;
disp('Vehicle data loaded');

%% Controller parameters

Nh = 5;
Beta = 1;

if Beta ==0
    q = 40;
    rc = 10;
else
    q =  45;
    rc = 10;
end