function [TMotor] = motorController(sampleTime,Torque_pre,omega,Torque_req)

motorSpeed = omega*16;

% Add torque vectoring
T_error = Torque_req-Torque_pre;     % Change from last time step

dT_max = 85.6*sampleTime ;  % Maximum change motors can produce in one time step

TMotor = Torque_pre + min(max(T_error,-dT_max),dT_max);  % Calc torque

maxTorque = zeros(2,1);
breakSpeed = motorSpeed(3:4) < 11000*pi/30;
maxTorque(breakSpeed) = 24;
maxTorque(~breakSpeed) = 38e3/motorSpeed(motorSpeed(3:4) >= 11000*pi/30);
maxTorque(motorSpeed(3:4) > 20e3*pi/30) = maxTorque(motorSpeed(3:4) > 20e3*pi/30)*0;
maxTorque = [0;0;maxTorque*16];

TMotor = max(min(TMotor,maxTorque),-maxTorque);
end