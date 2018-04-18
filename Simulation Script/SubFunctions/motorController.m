function [TMotor] = motorController(accReq,x,yawRef,sampleTime,Iz,wRadius,m,Torque_pre,omega)
gearRatio = 16;

r = x(3);
dr = x(6);
motorSpeed = omega*gearRatio;

% Add torque vectoring
e = (yawRef-r*sampleTime)*2/(sampleTime^2) - dr;
Mz = 0*e*Iz; % Disable for now

Torque = [0;0;1;1]*accReq*m/(2*wRadius) + [0;0;-1;1]*Mz/2; % Calc desired torque
    
T_error = Torque-Torque_pre;     % Change from last time step

dT_max = 50*sampleTime;  % Maximum change motors can produce in one time step

TMotor = Torque_pre + min(max(T_error,-dT_max),dT_max);  % Calc torque

maxTorque = zeros(2,1);
breakSpeed = motorSpeed(3:4) < 11000*pi/30;
maxTorque(breakSpeed) = 24;
maxTorque(~breakSpeed) = 38e3/motorSpeed(motorSpeed(3:4) >= 11000*pi/30);
maxTorque(motorSpeed(3:4) > 20e3*pi/30) = maxTorque(motorSpeed(3:4) > 20e3*pi/30)*0;
maxTorque = [0;0;maxTorque*gearRatio];

TMotor = max(min(TMotor,maxTorque),-maxTorque);
end