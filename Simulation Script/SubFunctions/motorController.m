function [Torque] = motorController(accReq,x,yawRef,sampleTime,Iz,wRadius,m)

r = x(3);
dr = x(6);
% Add torque vectoring
e = (yawRef-r*sampleTime)*2/(sampleTime^2) - dr;
Mz = 0*e*Iz;

Torque = [0;0;1;1]*accReq*m/(2*wRadius) + [0;0;-1;1]*Mz/2;

end