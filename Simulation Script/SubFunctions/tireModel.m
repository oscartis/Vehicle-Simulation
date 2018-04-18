function [Fy, alpha, Ku] = tireModel(delta,x,l1, l2, w, Fz,Fx, tireLoad, tireSlip, tireForce)
%% load states

u = x(1);
v = x(2);
r = x(3);


%% Friction coefficient
Fz0     = 1500;
mu0     = 2.1385;                    % Road tyre friction coefficient
mu1     = -2.3862e-04;
mu      = mu0*(1-mu1*(Fz-Fz0));


%% Understeer coefficient
Ca = 2e-15*(Fz).^6 -2e-11*(Fz).^5 + 5e-8*(Fz).^4 ... 
    - 6e-5*(Fz).^3 + 0.0066*(Fz).^2 + 53.121*(Fz) + 2.9346;
l = [l1;l1;l2;l2];

Ku    = ((Ca(3)+Ca(4))*l2-(Ca(1)+Ca(2))*l1)/((Ca(1)+Ca(2))*(Ca(3)+Ca(4))*(l1-l2));

%% calc tire force

Fy_max = sqrt(max(0.01,(Fz.*mu).^2-Fx.^2));       % Saturation limit

alpha = delta - atan2(v+l*r,u+w*r);     % Wheel slip
alpha = min(max(alpha,tireSlip(1)),tireSlip(end));


Fy = interp2(tireSlip,tireLoad,tireForce',alpha,Fz,'linear',Fz'/(4*tireLoad(1))*interp1(tireSlip,tireForce(:,1),alpha)); % Look-up table from 
                                                    % experimental data

Fy = min(max(Fy,-Fy_max),Fy_max);       % Saturate based on friction circle


