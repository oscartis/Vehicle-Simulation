function dx = motion(delta,Fy,Fx,m,Izz,l,w, x,sampleTime,rho,A,Cd)
% Input: Vector of vehicle states (x), road wheel angle (deltaf), vector of
% lateral and longitudinal wheel forces (Fy and Fx), vehicle parameters
% Output: Derivative of vector of vehicle states (dx)

% x - [Long. vel; Lat vel; Yaw rate; Long acc; Lat acc; Yaw acc]

u = x(1);
v = x(2);
r = x(3);
fr = 0.01;
g = 9.81;

%delta = deltaf;
% 
% %%=======================================================================%%
% % % Equations of motion

dx      = zeros(6,1);

Ffly = Fy(1)*cos(delta(1))+Fx(1)*sin(delta(1));
Ffry = Fy(2)*cos(delta(2))+Fx(2)*sin(delta(2));
Frly = Fy(3)*cos(delta(3))+Fx(3)*sin(delta(3));
Frry = Fy(4)*cos(delta(3))+Fx(3)*sin(delta(3));

Fflx = -Fy(1)*sin(delta(1))+Fx(1)*cos(delta(1));
Ffrx = -Fy(2)*sin(delta(2))+Fx(2)*cos(delta(2));
Frlx = -Fy(3)*sin(delta(3))+Fx(3)*cos(delta(3));
Frrx = -Fy(4)*sin(delta(4))+Fx(4)*cos(delta(4));

% Resistances
FxRes = m*g*fr + 1/2*rho*Cd*A*u^2;


du         = (Ffrx+Fflx+Frlx+Frrx - FxRes)/m + r*v;
dv         = (Ffry+Ffly+Frly+Frry)/m - r*u;
dr         = (l(3)*(Frry+Frly)+l(1)*(Ffry+Ffly)+w(1)*(Ffrx-Fflx)+w(3)*(Frrx-Frlx))/Izz;
%%=======================================================================%%

% % State derivatives
dx(1:3)     = [du; dv; dr];

% % Creating extra state derivates
dx(4)       = (du - v*r - x(4))*(1/sampleTime);
dx(5)       = (dv + u*r - x(5))*(1/sampleTime);
dx(6)       = (dr - x(6))*(1/sampleTime);

% These extra state derivates, when integrated give ax, ay and r_dot.
% This is done in order to overcome an algebraic loop problem. (i.e, load
% transfer depends on accelerations which depend on tyre forces which
% depend on load transfer again -- a chicken and egg problem).
