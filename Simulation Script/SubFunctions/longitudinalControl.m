function [Fx,omega] = longitudinalControl(Torque,x, Fz, FxBrakes,Iyw,omega,sampleTime,mu0,mu1,Fz0,wRadius)
u = x(1);

B =       1.621;
C =       9.307;
E =       14.81;

mu = mu0*(1-mu1*(Fz-Fz0));


slip = [0;0;0;0];
if u == 0
    slip(omega == 0) = 0;
    slip(omega ~= 0) = sign(omega(omega~=0))*1;
else
    slip = (omega*wRadius-u)/abs(u);
end


Fx = Fz.*mu.*sin(C*atan(B.*slip - E*(B.*slip - atan(B.*slip))));

omegaDot = (Torque + wRadius*(-Fx + FxBrakes))/Iyw;
omega = omega + omegaDot*sampleTime;

end