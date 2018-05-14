function [TMotor] = motorController(accReq,x,sampleTime,wRadius,m,Torque_pre,omega,rho,A,Cd,Mz)

r = x(3);
dr = x(6);
u = x(1);
motorSpeed = omega*16;

% Add torque vectoring
% e = (yawRef-r*sampleTime)*2/(sampleTime^2) - dr;
% Mz = 0*e*Iz; % Disable for now

Torque = ([0;0;1/2;1/2]*accReq*m + [0;0;1/2;1/2]*rho*A*Cd*u^2*0.5 + [0;0;1/2;1/2]*0.01*m*9.81+[0;0;-1;1]*Mz/1.2)*wRadius; %+ [0;0;-1;1]*Mz/2; % Calc desired torque
    
T_error = Torque-Torque_pre;     % Change from last time step

dT_max =50.3030*sampleTime*16;  % Maximum change motors can produce in one time step

TMotor = Torque_pre + min(max(T_error,-dT_max),dT_max);  % Calc torque

maxTorque = zeros(2,1);
breakSpeed = motorSpeed(3:4) < 10000*pi/30;
maxTorque(breakSpeed) = 24;
maxTorque(~breakSpeed) = 24*(2e4-motorSpeed(~breakSpeed)*30/pi)/1e4;
maxTorque(motorSpeed(3:4) > 20e3*pi/30) = maxTorque(motorSpeed(3:4) > 20e3*pi/30)*0;
maxTorque = [0;0;maxTorque*16];

TMotor = max(min(TMotor,maxTorque),-maxTorque);
end