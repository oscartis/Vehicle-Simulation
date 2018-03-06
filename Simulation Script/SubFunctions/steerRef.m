function delta= steerRef(r_ref,x,L,Ku,m)
maxDelta = pi/6;

%k = 0.8;
u = x(1);
%v = x(5);


deltaf  = (L+Ku*m*u^2)*r_ref/u;

delta = [1;1;0;0]*max(min(deltaf,maxDelta),-maxDelta);%*v*r;