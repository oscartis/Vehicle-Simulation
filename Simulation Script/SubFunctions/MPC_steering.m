function [delta,state_del,del_opt,eig_A ] = MPC_steering(Fz,x,x_1,m,l1,l2,Izz,sampleTime,r,q,state_previous_del,Nh,r_ref,del_opt_previous,delta_previous)

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
yaw = x_1(3);

a11 = 1-(C/m/u)*st;
a12 = -(1+(Cs/m/u^2))*st;
a21 = -(Cs/Izz)*st;
a22 = 1-(Cq2/Izz/u)*st;

b11 = C1/m/u;
b21 = C1*l1/Izz;

A = [a11,a12;a21,a22];
B = [b11;b21]*st;
CC = [0 1];

eig_A = eig(A);

H = zeros(Nh,Nh);
F = zeros(Nh,3);

M = [A B ; zeros(1,2) 1];
Q = [CC 0];
N = [B;1];

z=[0.1;0.3;0.5];

r_refe(1:Nh,1) = r_ref;
%%%%%%%%%%%%%%%% Finding state observer gain %%%%%%%%%%%%%%%%%%%%%%

a = M(1,1)+M(2,2)+M(3,3);
b = M(2,1);
c = -M(1,1)-M(3,3);
d = M(2,3);
e = M(1,1)*M(2,2) - M(1,2)*M(2,1) + M(1,1)*M(3,3) - M(1,3)*M(3,1) + M(2,2)*M(3,3) - M(2,3)*M(3,2);
f = -M(2,1)*M(3,3)+M(2,3)*M(3,1);
g = -M(1,3)*M(3,1)+M(1,1)*M(3,3);
h = -M(1,1)*M(2,3)+M(1,3)*M(2,1);
i = - M(1,3)*M(2,1)*M(3,2) - M(1,2)*M(2,3)*M(3,1) - M(1,1)*M(2,2)*M(3,3)+ M(1,3)*M(2,2)*M(3,1) + M(1,2)*M(2,1)*M(3,3) + M(1,1)*M(2,3)*M(3,2);

c1 = z*b+f;
c2 = z*c+g+z.^2;
c3 = z*d+h;
c4 =-(z.^3-z.^2*a+z*e+i);

Coeff_matrix = cat(2,c1,c2,c3);

L = inv(Coeff_matrix)*c4;

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

state_del= M*state_previous_del+L*(yaw-Q*state_previous_del)+N*del_opt_previous(1);

del_opt = (H'*R*H+Q_new)^(-1)*H'*R*(r_refe-F*state_del);

deltaf =[1;1;0;0]*del_opt(1)+delta_previous;

 delta = [1;1;0;0]*max(min(deltaf(1),maxDelta),-maxDelta);

end