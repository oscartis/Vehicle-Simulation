function [Fx,omega] = longitudinalControl(Torque, FxBrakes,Iyw,omega,sampleTime)
u = x(1);
slip = (omega - u)/(u);

Fx = interp2(tireLoad, tireSlipx, tireForcex,Fz,slip);

omegaDot = (-Fx+Torque+FxBrakes)/Iyw;
omega = omegaDot*sampleTime;

end