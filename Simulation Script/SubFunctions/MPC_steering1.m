function [delta,state_del,del_opt,eig_A,Mzf ] = MPC_steering1(Fz,x,x_1,m,l1,l2,Izz,sampleTime,r,q,state_previous_del,Nh,r_ref,del_opt_previous,delta_previous,Mz_previous)

Ca = 2e-15*(Fz).^6 -2e-11*(Fz).^5 + 5e-8*(Fz).^4 ...
    - 6e-5*(Fz).^3 + 0.0066*(Fz).^2 + 53.121*(Fz) + 2.9346;

maxDelta = 24*pi/180;

C1 = Ca(1)+Ca(2);
C2 = Ca(3)+Ca(4);
C = C1+C2;
Cs = l1*C1-l2*C2;
Cq2 = l1^2*C1+l2^2*C2;
st = sampleTime;
u = x(1);
v = x(2);
r_yaw = x(3);
r_1 = x_1(3);
v_1 = x_1(2);
u_1 = x_1(1);

a11 = 1-(C/m/u)*st;
a12 = -(1+(Cs/m/u^2))*st;
a21 = -(Cs/Izz)*st;
a22 = 1-(Cq2/Izz/u)*st;

b11 = C1/m/u;
b12 = 0;
b21 = C1*l1/Izz;
b22 = 1/Izz;

A = [a11,a12;a21,a22];
B = [b11 b12;b21 b22]*st;
CC = [1 0 ; 0 1];
% CC = [0 1];
eig_A = eig(A);
H = zeros(2*Nh,2*Nh);
F = zeros(2*Nh,4);
% H = zeros(Nh,Nh);
% F = zeros(Nh,3);

M = [A B ; zeros(2) eye(2)];
Q = [CC zeros(2)];
% Q = [CC 0];
N = [B;eye(2)];

% z=[0.1;0.3;0.5];
Z = [0.1 0; 0 0.2];
G = [1,2;5,6];

k = 0;
for i = 1:Nh
    r_refe(i+k) = 0;
    r_refe(i+k+1) = r_ref(i);
    k=k+1;
end

%%%%%%%%%%%%%%%% Finding state observer gain %%%%%%%%%%%%%%%%%%%%%%

eq = @(X) A'*X - X*Z - CC'*G;
X0 = ones(2);
options = optimoptions('fsolve','Display','none');
[X] = fsolve(eq,X0,options);
L_t = G*X^(-1);
L = L_t';

%%%%%%%%%%%%%%%% Finding state observer gain %%%%%%%%%%%%%%%%%%%%%%
%
%     a = M(1,1)+M(2,2)+M(3,3);
%     b = M(2,1);
%     c = -M(1,1)-M(3,3);
%     d = M(2,3);
%     e = M(1,1)*M(2,2) - M(1,2)*M(2,1) + M(1,1)*M(3,3) - M(1,3)*M(3,1) + M(2,2)*M(3,3) - M(2,3)*M(3,2);
%     f = -M(2,1)*M(3,3)+M(2,3)*M(3,1);
%     g = -M(1,3)*M(3,1)+M(1,1)*M(3,3);
%     h = -M(1,1)*M(2,3)+M(1,3)*M(2,1);
%     i = - M(1,3)*M(2,1)*M(3,2) - M(1,2)*M(2,3)*M(3,1) - M(1,1)*M(2,2)*M(3,3)+ M(1,3)*M(2,2)*M(3,1) + M(1,2)*M(2,1)*M(3,3) + M(1,1)*M(2,3)*M(3,2);
%
%     c1 = z*b+f;
%     c2 = z*c+g+z.^2;
%     c3 = z*d+h;
%     c4 =-(z.^3-z.^2*a+z*e+i);
%
%     Coeff_matrix = cat(2,c1,c2,c3);
%
%     L = inv(Coeff_matrix)*c4;

%%%%%%%%%%%%%%%%%%%%%%%%%% Controller Matrix %%%%%%%%%%%%%%%%%%%%%%%%%
k=0;
for j = 1:Nh
    %       F(j,:) = Q*M^j;
    F(j+k:j+k+1,:) = Q*M^j;
    k=k+1;
end

k=0;
for i=1:Nh
    for j=Nh:-1:i
        %         H(j,i)= Q*M^(j-i)*N;
        H(2*j-1:2*j,i+k:i+1+k)= Q*M^(j-i)*N;
    end
    k=k+1;
end

R = r*eye(2*Nh);
% R = r*eye(Nh);
Q_new = q*eye(2*Nh);

%  r_ref = r_ref.*(Nh:-1:1);

state_act = [atan2(v_1,u_1);r_1];
%  state_act = [r_yaw];

state_del= A*state_previous_del(1:2,:)+L*(state_act-CC*state_previous_del(1:2,:))+B*[delta_previous(1);Mz_previous];
state_del = [state_del;delta_previous(1);Mz_previous];
% state_del = [atan2(v,u);r_yaw;delta_previous(1)];

del_opt = (H'*R*H+Q_new)^(-1)*H'*R*(r_refe'-F*state_del);

deltaf =[1;1;0;0]*del_opt(1)+delta_previous;
Mzf =del_opt(2)+Mz_previous;
delta = [1;1;0;0]*max(min(deltaf(1),maxDelta),-maxDelta);

end
