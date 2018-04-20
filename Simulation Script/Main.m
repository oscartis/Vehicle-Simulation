addpath('SubFunctions')
InitializeModel;

n = simTime/sampleTime+1;

t = 0;
i = 2;

x = zeros(6,n);
x(:,2)      = [u0;0;0;0;0;0];
X           = zeros(3,n);
X(:,2)      = [X0;Y0;Psi0];
xDot        = zeros(6,n);
phi         = zeros(1,n);
phiDot      = phi;
phiDDot     = phi;
r_ref       = phi;
delta       = zeros(4,n);
alpha       = delta;

headingRequest  = phi;

accelerationRequest = [5,zeros(1,n-1)];
aimPoint        = zeros(2,n);

Fy          = delta;
Fx          = delta;
preFx       = zeros(4,1);
Fz          = delta;
state_del   = zeros(3,n);
del_opt     = zeros(Nh,n);
state_a     = zeros(2,n);
a_opt       = zeros(Nh,n);
r_ref       = zeros(n,1);
Torque      = delta;

%% Controller parameters

q = 10;
rc = 15;
Nh = 5;
state_del   = zeros(3,n);
del_opt     = zeros(Nh,n);
state_a     = zeros(2,n);
a_opt       = zeros(Nh,n);

wheelLiftOffFlag = zeros(1,n);
spin = 0;
omega = u0/wRadius*[1;1;1;1];

Torque_req      = delta;
Torque_dot  = zeros(n,1);
Torque_req(:,1) = [0;0;1;1]*0.01*9.81*m;
%%

while t  <= simTime
    

   [headingRequest(i), localPath, aimPoint(:,i)] = ...
       genAimPoint(X(:,i),x(:,i),trackPath);

    
    r_ref(i)    = ...
         yawModel(headingRequest(i),x(:,i));
    
%       delta(:,i)  = ...
%          steerRef(r_ref(i),x(:,i),L,Ku,m);

       [delta(:,i),state_del(:,i),del_opt(:,i),eig_A(:,i) ] = MPC_steering(Fz(:,i),x(:,i),x(:,i-1),m,l1,-l2,Izz,sampleTime,rc,q,state_del(:,i-1),Nh,r_ref(i),del_opt(:,i),delta(:,i-1));
    
   if abs(x(3,i)) > 1
        spin = 1;
    elseif abs(x(3,i)) < 0.2
        spin = 0;
    end
    

    [Fz(:,i), phiDDot(i), wheelLiftOffFlag(i)] = ...
        loadTransfer(x(:,i),phi(i),phiDot(i),m, ms, Ixx, hp, kLambda, kPhi, cLambda, ...
        cPhi, g, h2, h1, h0, h, w, L,l,A, Cd, Cl, rho);
    
    [accelerationRequest(i)] = ...
        getAccReq(x(1,i), velocityLimit, lateralAccelerationLimit, ...
        accelerationLimit, decelerationLimit, headingRequest(i), ...
        headingErrorDependency, localPath');


    [Torque_dot(i),Torque_req(:,i),state_a(:,i),a_opt(:,i)] = MPC_velocity(x(:,i),x(:,i-1),m,sampleTime,wRadius,G_ratio,Nh,rc,q,state_a(:,i-1),a_opt(:,i-1),Torque_req(:,i-1),accelerationRequest(i),Torque_dot(i-1));  
    [Torque(:,i)] = motorController(accelerationRequest(i),x(:,i),r_ref(i),sampleTime,Izz,wRadius,m,Torque(:,i-1),omega,Torque_req(:,i));
    
    [FxBrakes] = brakes(accelerationRequest(i),m);
    
    [Fx(:,i), omega] = ...
        longitudinalControl(Torque(:,i), x(:,i), Fz(:,i), FxBrakes, Iyw, omega,sampleTime,tireLoad,tireSlipX,tireForceX,wRadius);
        
    Fx(:,i) =  ...
        Fx(:,i)*(1-spin);
%     Fx(:,i) = [0;0;0;0];
    
    [Fy(:,i), alpha(:,i), Ku]  = ...
        tireModel(delta(:,i),x(:,i),l1, l2, w, Fz(:,i),Fx(:,i),tireLoad,tireSlipY,tireForceY);
    
    dx = ...
        motion(delta(:,i),Fy(:,i),Fx(:,i),m,Izz,l,w, x(:,i),sampleTime,rho,A,Cd);
    
    xDot(:,i) = dx;
    
    %% Integrate states
    
    if i <= simTime/sampleTime
        
        x(:,i+1) = x(:,i) + dx*sampleTime;
        
        
        X(3,i+1)    = X(3,i) + x(3,i)*sampleTime + dx(3)*sampleTime^2/2;
        
        R = [cos(X(3,i+1)), -sin(X(3,i+1));
            sin(X(3,i+1)), cos(X(3,i+1))];
        
        X(1:2,i+1)  = X(1:2,i) + R*x(1:2,i)*sampleTime + R*dx(1:2)*sampleTime^2/2;
        
        phi(i+1) = phi(i) + phiDot(i)*sampleTime + phiDDot(i)*sampleTime^2/2;
        phiDot(i+1) = phiDot(i) +phiDDot(i)*sampleTime;
        
        i = i+1;
    end
    
    if mod(i,100) == 0
        fprintf('Running: %2.1f \n',t/simTime*100)
    end
    
    t = t + sampleTime;
end

Y   = X(2,:);
Psi = X(3,:);
X   = X(1,:);

t = (0:sampleTime:simTime);

GeneratePlots;
