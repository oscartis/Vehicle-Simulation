function r_ref = yawModel(headingReq,x)

u = x(1);
d = 5;

R = d/(sqrt(2*(1-cos(2*headingReq))));

r = sign(headingReq)*u/R;

r_ref = r;