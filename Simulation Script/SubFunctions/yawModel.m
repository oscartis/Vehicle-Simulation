function [r_ref,R] = yawModel(headingReq,x,Nh,CR,i)

u = x(1);

d = 5;

R = d/(sqrt(2*(1-cos(2*headingReq))));

r_ref(1,1:Nh) = sign(headingReq)*u/R;


