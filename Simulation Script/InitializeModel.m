%% Some clean up

%clc;
clear all;
close all;

%% %%% Test case definiton %%%%%%%%%%%%%%%%%%%%%%%%%%
u0          = 5;          % Longitudinal Speed [m/s]

trackPath   = load('trackReconstructed.mat');
trackPath   = trackPath.trackReconstructed;

X0          = trackPath(1,1); 
Y0          = trackPath(1,2);
X1          = trackPath(40,1); 
Y1          = trackPath(40,2);
Psi0        = -2.6366;%atan2(X1-X0,Y1-Y0);

%% %%%% Simulation parameters %%%%%%%%%%%%%%%%%%%%%%%
sampleTime  = 0.01;             % Simulation Step Size [s]
simTime     = 10;               % Simulation end time [s]

%% %%% Car parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
m       = 217.4;                % Mass [kg]
Izz     = 115;                  % Moment of inertia about z axis [kgm^2]
L       = 1.53;                 % Wheelbase [m]
l1      = 0.55*L;               % Distance from COG to front axle [m]
l2      = l1-L;                 % Distance from COG to rear axle [m]
l       = [l1;l1;l2;l2];
w       = [1.25;-1.25;1.2;-1.2]/2;  % Track width [m]
m_us    = 28;
ms      = m-m_us;               % Sprung mass [kg]
Ixx     = 100;                  % Vehicle inertia about X axis
cPhi    = 7e4;                  % Vehicle total roll stiffness
kPhi    = 8000;                 % Vehicle total roll damping
cLambda = 0.55;                 % Vehcile roll stiffness distribution
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
mu0     = 1;                    % Road tyre friction coefficient
mu1     = 6e-5;                 % Tyre load based non-linearity parameter for friction
c0      = 21.3/2;               % Tyre stiffness parameter
c1      = 1.11e-4;              % Tyre load based non-linearity parameter for stiffness
Fz0     = 4000;                 % Rated load for the tyres
tvFrcLim= 2500;                 % Force limit for torque vectoring (each wheel)

%% Cornering stiffness
Fz      = g*[m*-l2/L m*l1/L]/2;
mu      = mu0*(1-mu1*(Fz-Fz0));
Ca1     = 20e3;
Ca2     = 20e3;
% c       = c0*(1-c1*(Fz-Fz0));
% B       = c./(mu*C);

D       = mu;   %[703.7844  454.6802]
B       = [Ca1 Ca2]./(C*D);


% Ca1 = B(1)*C*D(1);
% Ca2 = B(2)*C*D(2);
% ku = (-l2/L*m/Ca1-l1/L*m/Ca2);

disp('Vehicle data loaded');
