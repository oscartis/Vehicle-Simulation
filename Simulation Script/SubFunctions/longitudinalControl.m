function [Fx, preFx] = longitudinalControl(x,accReq,m,Fz,mu0,mu1,Fz0,Fx_pre,sampleTime)

u = x(1);

% dF/0.05 = 200 N

mu = mu0*(1-mu1*(Fz-Fz0));


fx = accReq*m;

[FxMotor] = motorControl(fx);
[FxHydraulic] = hydraulicBrake(fx);

Fx = FxMotor;
Fx = Fx + [0.75;0.75;0.25;0.25]*FxHydraulic/2;


Fx = max(min(mu.*Fz,Fx),-mu.*Fz);
preFx = FxMotor;


function [FxMotor] = motorControl(fx)
    dF_max = 200 * sampleTime * 20;
    
    e = fx-sum(Fx_pre);
    
    FxMotor = Fx_pre + min(max(e,-dF_max),dF_max)*[0;0;1;1];
end

function [FxHydraulic] = hydraulicBrake(fx)
    FxHydraulic = fx;
end

end