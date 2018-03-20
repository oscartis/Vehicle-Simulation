function [TMotor] = motorController(accReq,x,yawRef,sampleTime,Iz,wRadius,m)

r = x(3);
dr = x(6);

% Add torque vectoring
e = (yawRef-r*sampleTime)*2/(sampleTime^2) - dr;
Mz = 0*e*Iz; % Disable for now

Torque = [0;0;1;1]*accReq*m/(2*wRadius) + [0;0;-1;1]*Mz/2; % Calc desired torque
    
T_error = Torque-Torque_pre;     % Change from last time step

dT_max = 20 * sampleTime * 20;  % Maximum change motors can produce in one time step

TMotor = Fx_pre + min(max(T_error,-dT_max),dT_max);  % Calc torque
end