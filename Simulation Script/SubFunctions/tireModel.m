function [Fy, alpha] = tireModel(delta,x,l1, l2, w,B, C, D, E, Fz)

l = [l1;l1;-l2;-l2];

u = x(1);
v = x(2);
r = x(3);
B = [B(1);B(1);B(2);B(2)];
D = [D(1);D(1);D(2);D(2)];

alpha = delta - atan2(v+l*r,u+w*r);

Fy = Fz.*D.*sin(C*atan((B.*alpha - E*(B.*alpha - atan(B.*alpha)))));
