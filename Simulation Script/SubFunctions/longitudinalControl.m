function [Fx,omega] = longitudinalControl(Torque,x, Fz, FxBrakes,Iyw,omega,sampleTime,tireLoad,tireSlipX,tireForceX)
u = x(1);

if u == 0
    slip(omega == 0) = 0;
    slip(omega ~= 0) = tireSlipX(end);
else
    slip = (omega*wRadius-u)/u;
end

Fx = interp2(tireLoad, tireSlipX, tireForceX,Fz,slip');

omegaDot = (-Fx+Torque+FxBrakes)/Iyw;
omega = omegaDot*sampleTime;

end