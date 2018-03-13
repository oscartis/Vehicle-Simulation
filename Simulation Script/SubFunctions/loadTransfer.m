function [Fz, phiDDot, wheelLiftOffFlag] = loadTransfer(x,phi,phiDot,m, ms, Ixx, hp, kLambda, kPhi, cLambda, cPhi, g, h2, h1, h0, h, w, L,l,A, Cd, Cl, rho)
% Inputs: Vehicle states (x), roll angle and roll rate (phi and phiDot),
% vehicle parameters
% Outputs: Wheel normal loads (Fz), roll acceleration (phiDDot), flag
% indicating wheel lift-off (wheelLiftOffFlag)
u       = x(1);
ax      = x(4);
ay      = x(5);

cPhi1   = cPhi*cLambda;
cPhi2   = cPhi - cPhi1;
kPhi1   = kPhi*kLambda;
kPhi2   = kPhi - kPhi1;

Fl      = 1/2*A*rho*Cl*u^2;
Fd      = 1/2*A*rho*Cd*u^2;

FzStatic = (Fl + m*g)/L/2*[-l(3);-l(3);l(1);l(1)];

dFz=[0;0;0;0];

%%=======================================================================%%
% load transfer for roll and steady state load transfer for pitch
phiDDot = 0;%(ms*ay*h0-kPhi*phiDot-(cPhi)*phi+ms*g*h0*sin(phi))/(Ixx+ms*h0^2);

dFzy = [-1/w(1)*(m*(-l(3))*ay*h1/L/2) + (cPhi1*phi+kPhi1*phiDot), ...
    -1/w(2)*(m*(-l(3))*ay*h1/L/2) - (cPhi1*phi+kPhi1*phiDot), ...
    -1/w(3)*(m*(l(1))*ay*h2/L/2) + (cPhi2*phi+kPhi2*phiDot), ...
    -1/w(4)*(m*(l(1))*ay*h2/L/2) - (cPhi2*phi+kPhi2*phiDot)]';

dFzx = [-m*ax*h/2/L - Fd*hp/2/L; -m*ax*h/2/L - Fd*hp/2/L; ...
    +m*ax*h/2/L + Fd*hp/2/L; +m*ax*h/2/L + Fd*hp/2/L];

Fz      = FzStatic + dFzx + dFzy;


%%=======================================================================%%

% % Wheel lift-off scenario. This is not really *handling* wheel lift-off,
% but rather making sure the simulation does not crash in the event of
% wheel lift-off. Solution may still be incorrect. Handling wheel lift-off
% is not straight forward. So this simulation model is not good for cases
% with excessive or frequent wheel lift-off.
eps     = 1e-3;
wheelLiftOffFlag = 0;
if any(Fz < 0)
    wheelLiftOffFlag = 1;
end
Fz(Fz < eps)  = eps;
Fz = Fz*((m*g+Fl)/sum(Fz));