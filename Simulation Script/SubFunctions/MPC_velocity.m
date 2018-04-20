function [Torque_dot,Torque_req,state_a,a_opt] = MPC_velocity(x,x_1,m,sampleTime,wRadius,G_ratio,Nh,r,q,state_a_previous,a_opt_previous,Torque_req_previous,acc_req,Torque_dot_previous)

if acc_req >=0
    
    a_ref = zeros(Nh,1);
    a_ref(1:Nh,1) = acc_req;
    
    u = x(1);
    a = x_1(4);
    pseudo = 1;
    m = pseudo*m;
    Cd = 1.21;
    Ar = 1.14;
    rho = 1.18415;
    
    A = 1-(rho*Cd*Ar*u*sampleTime/m);
    B = 2*sampleTime/wRadius/m;
    
    M = [A B ; 0 1];
    N = [B;1];
    Q = [1 0];
    
    eig_M = eig(M);
    z = [0.1 ; 0.5];
    
    %%%%%%%%%%%%%%%% Finding state observer gain %%%%%%%%%%%%%%%%%%%%%%
    
    L =[M(1,1) - z(2) - z(1) + M(2,2);(z(1)*z(2) - z(1)*M(2,2) - z(2)*M(2,2) + M(1,2)*M(2,1) + M(2,2)^2)/M(1,2)];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%% Controller Matrix %%%%%%%%%%%%%%%%%%%%%%%%%
    
    for j = 1:Nh
        F(j,:) = Q*M^j;
    end
    
    for i=1:Nh
        for j=Nh:-1:i
            H(j,i)= Q*M^(j-i)*N;
        end
    end
    
    R = r*eye(Nh);
    Q_new = q*eye(Nh);
    
    state_a = M*state_a_previous+L*(a-Q*state_a_previous)+N*a_opt_previous(1);
    a_opt = (H'*R*H+Q_new)^(-1)*H'*R*(a_ref-F*state_a);
    Torque_dot = a_opt(1)+ Torque_dot_previous;
    
    Torque_req = Torque_req_previous + [0;0;1;1]*Torque_dot*sampleTime ;
    Torque_req = min(Torque_req,[0;0;24*G_ratio;24*G_ratio]);
    
else
    Torque_dot = 0;
    a_opt = 0;
    Torque_req = [0;0;0;0];
    state_a = [x(4);0];
end
    
end