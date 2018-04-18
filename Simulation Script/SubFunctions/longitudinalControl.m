function [Fx,omega] = longitudinalControl(Torque,x, Fz, FxBrakes,Iyw,omega,sampleTime,tireLoad,tireSlipX,tireForceX,wRadius)
u = x(1);

slip = [0;0;0;0];
if u == 0
    slip(omega == 0) = 0;
    slip(omega ~= 0) = tireSlipX(end);
else
    slip = (omega*wRadius-u)/abs(u);
end

slip = min(max(slip,tireSlipX(1)),tireSlipX(end));

Fx = interp2(tireLoad,tireSlipX, tireForceX,Fz,slip,'linear',Fz'/(4*tireLoad(1))*interp1(tireSlipX,tireForceX(:,1),slip));

omegaDot = (Torque + wRadius*(-Fx + FxBrakes))/Iyw;
omega = omega + omegaDot*sampleTime;

end