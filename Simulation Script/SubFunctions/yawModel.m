function r_ref = yawModel(velPoint,x)

u = x(1);
d = 2;

headingReq = atan2(velPoint(2),velPoint(1));

R = d/(sqrt(2*(1-cos(2*headingReq))));

r = sign(headingReq)*u/R;

r_ref = r;