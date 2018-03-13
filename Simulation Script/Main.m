addpath('SubFunctions')
InitializeModel;

n = simTime/sampleTime+1;

t = 0;
i = 1;

x = zeros(6,n);
x(:,1)      = [u0;0;0;0;0;0];
X           = zeros(3,n);
X(:,1)      = [X0;Y0;Psi0];
phi         = zeros(1,n);
phiDot      = phi;
phiDDot     = phi;
r_ref       = phi;
delta       = zeros(4,n);
alpha       = delta;

headingRequest  = phi;
velPoint        = zeros(2,n);
aimPoints       = zeros(2,n,2);
v_ref           = phi;
xDot            = zeros(6,n);

Fy          = delta;
Fx          = delta;
Fz          = delta;

wheelLiftOffFlag = zeros(1,n);
spin = 0;

%%

while t  <= simTime
    
    if abs(x(3,i)) > 1 
        spin = 1;
    elseif abs(x(3,i)) < .4
        spin = 0;
    end
    
    [headingRequest(i), velPoint(:,i), aimPoints(:,i,:)] = genAimPoint(X(:,i),x(:,i),trackPath);
    r_ref(i)            = yawModel(velPoint(:,i),x(:,i));
    delta(:,i)          = steerRef(headingRequest(i),x(:,i),L,Ku,m);
    
    [Fz(:,i), phiDDot(i), wheelLiftOffFlag(i)] = ...
        loadTransfer(x(:,i),phi(i),phiDot(i),m, ms, Ixx, hp, kLambda, kPhi, cLambda, ...
        cPhi, g, h2, h1, h0, h, w, L,l,A, Cd, Cl, rho);

    [Fx(:,i), v_ref(i)] = longitudinalControl(x(:,i),r_ref(i),m,L, l,Fz(:,i),mu0,mu1,Fz0);
    Fx(:,i)             = Fx(:,i)*(1-spin);
    [Fy(:,i), alpha(:,i), Ku]  = tireModel(delta(:,i),x(:,i),l1, l2, w, Fz(:,i),Fx(:,i),tireLoad,tireSlip,tireForce);
    dx                  = motion(delta(:,i),Fy(:,i),Fx(:,i),m,Izz,l,w, x(:,i),sampleTime);
    xDot(:,i) = dx;
    
    %% Integrate states
    
    if t < simTime
        x(:,i+1) = x(:,i) + dx*sampleTime;
        
        
        X(3,i+1)    = X(3,i) + x(3,i)*sampleTime + dx(3)*sampleTime^2/2;
        
        R = [cos(X(3,i+1)), -sin(X(3,i+1));
             sin(X(3,i+1)), cos(X(3,i+1))];
        
        X(1:2,i+1)  = X(1:2,i) + R*x(1:2,i)*sampleTime + R*dx(1:2)*sampleTime^2/2;
        
        phi(i+1) = phi(i) + phiDot(i)*sampleTime + phiDDot(i)*sampleTime^2/2;
        phiDot(i+1) = phiDot(i) +phiDDot(i)*sampleTime;
        
        i = i+1;
    end
    
    t = t + sampleTime;
end

Y   = X(2,:);
Psi = X(3,:);
X   = X(1,:);

t = (0:sampleTime:simTime);

GeneratePlots;
