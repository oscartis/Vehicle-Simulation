function [Fx, omega] = wheelMotion(Torque,x,Iyw,omega,Fx,sampleTime)

u = x(1);

omegaDot = (Torque - Fx)/Iyw;
omega = omega + omegaDot*sampleTime;

slip = (omega*wRadius-u)/u;



end