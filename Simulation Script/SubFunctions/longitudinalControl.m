function [Fx, v_ref] = longitudinalControl(x,yawRef,m,L, l,Fz,mu0,mu1,Fz0)

u = x(1);
r = x(3);


mu = mu0*(1-mu1*(Fz-Fz0));
u_min = 3/3.6;
u_max = 70/3.6;

k = 3;
v = 1/(k*abs(yawRef)+1)*u_max;

v_ref = max(u_min,v);



e = v_ref - u;

fx = e*m;

if fx > 0
    Fx = [0;0;1;1]*fx/2;
else
    Fx = [-l(3)/L;-l(3)/L;L(1)/L;l(1)/L]*fx/2;
end

Fx = min(mu.*Fz,Fx);
