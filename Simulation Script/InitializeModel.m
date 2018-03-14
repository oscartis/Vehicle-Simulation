%% Some clean up

%clc;
clear all;
close all;

%% %%% Test case definiton %%%%%%%%%%%%%%%%%%%%%%%%%%
u0          = 10;          % Longitudinal Speed [m/s]

trackPath   = load('TrackPath_smooth.mat');
trackPath   = trackPath.TrackPath_smooth;

X0          = trackPath(1,1); 
Y0          = trackPath(1,2);
X1          = trackPath(40,1); 
Y1          = trackPath(40,2);
Psi0        = 2.8102;%atan2(X1-X0,Y1-Y0);

%% %%%% Simulation parameters %%%%%%%%%%%%%%%%%%%%%%%
sampleTime  = .01;             % Simulation Step Size [s]
simTime     = 120;               % Simulation end time [s]

%% %%% Car parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
m       = 217.4;                % Mass [kg]
Izz     = 133.32;                  % Moment of inertia about z axis [kgm^2]
L       = 1.53;                 % Wheelbase [m]
l1      = 0.55*L;               % Distance from COG to front axle [m]
l2      = l1-L;                 % Distance from COG to rear axle [m]
l       = [l1;l1;l2;l2];
w       = [1.25;-1.25;1.2;-1.2]/2;  % Track width [m]
wRadius = 0.22;
m_us    = 28;
ms      = m-m_us;               % Sprung mass [kg]
Ixx     = 30.031;                  % Vehicle inertia about X axis
cPhi    = (2.2300e+04 + 1.7997e+04);                  % Vehicle total roll stiffness
kPhi    = 0;                 % Vehicle total roll damping
cLambda = 0.5534;                 % Vehcile roll stiffness distribution
kLambda = 0.57;                 % Vehicle roll damping distribution

h       = 0.282;                % Height of CoG
h1      = 0.040;                % Front roll center height
h2      = 0.093;                % Rear roll center height
h0      = h - (l1*h2 - l2*h1)/L;
                                % Height of CoG above roll axis
hp      = h + 0.15;
A       = 1.14;                 % Frontal area
rho     = 1.18415;              % density of air
Cd      = 1.21;                 % Drag coefficient
Cl      = 2.83*0.6;             % Lift coefficient
g       = 9.81;                 % Acceleration due to gravity
uESCLim = 7/3.6;                % Minimum speed for ESC to function
rErrLim = 0;                    % Yaw rate error threshold for ESC
escK    = 0;                    % Yaw rate gain for ESC
C       = 1.9;                  % Magic formula parameter
E       = 1;                    % Magic formula parameter
c0      = 21.3/2;               % Tyre stiffness parameter
c1      = 1.11e-4;              % Tyre load based non-linearity parameter for stiffness
Fz0     = 1500;                 % Rated load for the tyres
tvFrcLim= 2500;                 % Force limit for torque vectoring (each wheel)

%%
velocityLimit = 70/3.6;
lateralAccelerationLimit = g*0.75;
accelerationLimit = 0.8*g;
decelerationLimit = -g;
headingErrorDependency = 0.4;


%% Cornering stiffness
Fz = m*g*[-l2;-l2;l1;l1]/(2*L);

Ca = 2e-15*(Fz).^6 -2e-11*(Fz).^5 + 5e-8*(Fz).^4 ... 
    - 6e-5*(Fz).^3 + 0.0066*(Fz).^2 + 53.121*(Fz) + 2.9346;

Ku = ((Ca(3)+Ca(4))*l2-(Ca(1)+Ca(2))*l1)/((Ca(1)+Ca(2))*(Ca(3)+Ca(4))*(l1-l2));

Fz0     = 1500;
mu0     = 2.1385*0.66;                    
mu1     = -2.3862e-04;
mu      = mu0*(1-mu1*(Fz-Fz0));

%% Tire Data
load TireData
tireLoad = TireData(1,2:end);
tireLoad = -tireLoad;
%tireLoad(end) = [];
tireSlip = TireData(2:end,1)*pi/180;
tireForce = -TireData(2:end,2:end);
%tireForce(:,end) = [];
tireForce = tireForce;

disp('Vehicle data loaded');
