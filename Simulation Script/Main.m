addpath('SubFunctions')
InitializeModel;

n = simTime/sampleTime+1;

t = 0;
i = 2;

x           = zeros(6,n);
xp          = x;
x(1,1)      = u0;
x(:,2)      = [u0;0;0;0;0;0];
X           = zeros(3,n);
X(:,2)      = [X0;Y0;Psi0];
xDot        = zeros(6,n);
phi         = zeros(1,n);
phiDot      = phi;
phiDDot     = phi;
r_ref       = zeros(n,Nh);
Rad         = phi;
delta       = zeros(4,n);
alpha       = delta;
CR          = zeros(Nh,n);

headingRequest  = phi;
accelerationRequest = [5,zeros(1,n-1)];
aimPoint        = zeros(2,n);

Fy          = delta;
Fx          = delta;
preFx       = zeros(4,1);
Fz          = delta;
state_del   = zeros(4,n);
del_opt     = zeros(2*Nh,n);
state_a     = zeros(2,n);
a_opt       = zeros(Nh,n);
r_ref       = zeros(n,Nh);
Torque      = delta;
Mz          = zeros(n,1);
HD          = zeros(n,Nh);


wheelLiftOffFlag = zeros(1,n);
spin = 0;
omega = delta;
omega(:,2) = u0/wRadius*[1;1;1;1];
latDeviation = phi;
d = zeros(n,1);
var = 0;
%%

while t  <= simTime
    
    if ifStepResponse
        r_ref(i,:) = .5*(t>=8);
        %    r_ref(i,(t+sampleTime*(1:Nh)>8)) = 1;

        accelerationRequest(i) = (targetSpeed-x(1,i));
        Rad(i) = x(1,i)/r_ref(i,1);
    elseif ifSWD
        r_ref(i,:) = yawRateReference(i);
        accelerationRequest(i) = (targetSpeed-x(1,i));
        Rad(i) = x(1,i)/r_ref(i,1);
    else
        [headingRequest(i), localPath, aimPoint(:,i),d(i), cPi, cP_onTrack(i,:),curveRadius] = ...
            genAimPoint(X(:,i),x(:,i),trackPath);
        
        [r_ref(i,:),Rad(i)]    = ...
            yawModel(headingRequest(i),x(:,i),Nh,CR(:,i),i);
    
        if ifDLC 
            accelerationRequest(i) = (targetSpeed-x(1,i));
           
        else
        
        [accelerationRequest(i)] = ...
            getAccReq(x(1,i), velocityLimit, lateralAccelerationLimit, ...
            accelerationLimit, decelerationLimit, headingRequest(i), ...
            headingErrorDependency, localPath,curveRadius);
        end
    end
    
    [Fz(:,i), phiDDot(i), wheelLiftOffFlag(i)] = ...
        loadTransfer(x(:,i),phi(i),phiDot(i),m, ms, Ixx, hp, kLambda, kPhi, cLambda, ...
        cPhi, g, h2, h1, h0, h, w, L,l,A, Cd, Cl, rho);
    
    %         [r_ref(i,:),HD(i,:)]= curveEst(localPath,x(:,i),sampleTime,Nh,X(:,i));
    
%     delta(:,i)  = ...
%         steerRef(r_ref(i,1),x(:,i),L,Ku,m);
    
    if Beta ==1
        [delta(:,i),state_del(:,i),del_opt(:,i),eig_A(:,i),Mz(i) ] = MPC_steering1(Fz(:,i),x(:,i),x(:,i-1),m,l1,-l2,Izz,sampleTime,rc,q,state_del(:,i-1),Nh,r_ref(i,:),del_opt(:,i),delta(:,i-1),Mz(i-1));    
    else
        [delta(:,i),state_del(:,i),del_opt(:,i),eig_A(:,i),Mz(i)] = MPC_steering2(Fz(:,i),x(:,i),x(:,i-1),m,l1,-l2,Izz,sampleTime,rc,q,state_del(:,i-1),Nh,r_ref(i,:),del_opt(:,i),delta(:,i-1),Mz(i-1));
        
    end
    if abs(x(3,i)) > 1
        spin = 1;
    elseif abs(x(3,i)) < 0.2
        spin = 0;
    end
    
    
    %     [Torque_dot(i),Torque_req(:,i),state_a(:,i),a_opt(:,i)] = MPC_velocity(x(:,i),x(:,i-1),m,sampleTime,wRadius,G_ratio,Nh,rc,q,state_a(:,i-1),a_opt(:,i-1),Torque_req(:,i-1),accelerationRequest(i),Torque_dot(i-1));
    [Torque(:,i)] = motorController(accelerationRequest(i),x(:,i),sampleTime,wRadius,m,Torque(:,i-1),omega(:,i),rho,A,Cd,Mz(i));
    
    if ifSWD 
        Torque(:,i) = Torque(:,i)*(t<10);
    elseif ifDLC
        Torque(:,i) = Torque(:,i)*(X(1,i)<0);
    end

    [FxBrakes] = brakes(accelerationRequest(i),m);
    
    [Fx(:,i), omega(:,i+1)] = ...
        longitudinalControl(Torque(:,i), x(:,i), Fz(:,i), FxBrakes, Iyw, omega(:,i),sampleTime,mu0,mu1,Fz0,wRadius);
    
    %     Fx(:,i) =  ...
    %         Fx(:,i)*(1-spin);
    %     Fx(:,i) = [0;0;0;0];
    
    [Fy(:,i), alpha(:,i), Ku]  = ...
        tireModel(delta(:,i),x(:,i),l1, l2, w, Fz(:,i),Fx(:,i),tireLoad,tireSlipY,tireForceY,mu0,mu1,Fz0);
    
    dx = ...
        motion(delta(:,i),Fy(:,i),Fx(:,i),m,Izz,l,w, x(:,i),sampleTime,rho,A,Cd);
    
    xDot(:,i) = dx;
    
    if ~ifStepResponse && ~ifSWD
        [latDeviation(i)] = norm(X(1:2,i)-cP_onTrack');
    end
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

    if mod(round(t/sampleTime)/simTime*sampleTime*100,5) < 1e-5 %|| abs(mod(t/simTime*100,5)) >5-1e-5
        fprintf('Running: %2.1f%%\n',t/simTime*100)
    end

if ~ifStepResponse && ~ifDLC && ~ifSWD
    if t>10 && trackPath(cPi,1) == trackPath(1,1) && trackPath(cPi,2) == trackPath(1,2) && d(i)<0 && var ==0
         Laptime = Laptime_calculator(cP_onTrack,t,d,i,sampleTime,cPi,trackPath);
        var=1;
     end
end

    t = t + sampleTime;
end

Y       = X(2,:);
Psi     = X(3,:);
X       = X(1,:);
u       = x(1,:);
v       = x(2,:);
r       = x(3,:);
ax      = x(4,:);
ay      = x(5,:);
rDot    = x(6,:);

t = (0:sampleTime:simTime);

GeneratePlots;
