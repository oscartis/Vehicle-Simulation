function delta= steerRef(r_ref,x)
maxDelta = pi/6;

k = 0.8;

v = x(4);
r = x(6);


delta = [1;1;0;0]*max(min(k*r_ref,maxDelta),-maxDelta);%*v*r;