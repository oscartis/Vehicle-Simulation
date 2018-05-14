function [localPath, aimPoint, curveRadius] = curveEstimation(localPath,aimDist)
DoP = 7;
if DoP < 1 || mod(DoP,1) ~= 0
    error('Invalid DoP')
end
if ~exist('localPath','var')
    error('No path')
end

if length(localPath)<DoP
    error('DoP is greater than points in local path')
end

x = localPath(:,1);
y = localPath(:,2);

X = zeros(length(x),DoP);
X(:,1) = X(:,1)+1;

for i = 2:DoP
    X(:,i) = x.^(i-1);
end
C = (X'*X)\X'*y;
localPath = [x, X*C];

if DoP > 1
    yDot = X(:,1:end-1)*((1:length(C)-1)'.*C(2:end));
    if DoP > 2
        yDDot = X(:,1:end-2)*((1:length(C)-2)'.*(2:length(C)-1)'.*C(3:end));
    else
        yDDot = 0;
        warning('Low degree of polynom, bad estimate of curve radius')
    end
else
    yDot = 0;
    yDDot = 0;
    warning('Low degree of polynom, bad estimate of curve radius')
end

curveRadius = ((1+yDot.^2).^(3/2))./(abs(yDDot));


aimPoint = getPoint(aimDist,C,x(1));

    function [aimpoint] = getPoint(dist,C,x)
        a=x;
        f_dot = @(z) 11*22680*C(7)^2*z^10 + 10*41580*C(6)*C(7)*z^9 + 9*36960*C(5)*C(7)*z^8 + 9*19250*C(6)^2*z^8 + 8*34650*C(5)*C(6)*z^7 + 8*31185*C(4)*C(7)*z^7 + 7*29700*C(4)*C(6)*z^6 + 7*23760*C(3)*C(7)*z^6 + 7*15840*C(5)^2*z^6 + 6*27720*C(4)*C(5)*z^5 + 6*23100*C(3)*C(6)*z^5 + 6*13860*C(2)*C(7)*z^5 + 5*22176*C(3)*C(5)*z^4 + 5*13860*C(2)*C(6)*z^4 + 5*12474*C(4)^2*z^4 + 4*20790*C(3)*C(4)*z^3 + 4*13860*C(2)*C(5)*z^3 + 3*13860*C(2)*C(4)*z^2 + 3*9240*C(3)^2*z^2 + 2*13860*C(2)*C(3)*z + 6930*C(2)^2 + 6930;
        zs(1) = 7;
        zs(2) = 8;
        s=dist;
        tol = 3;
        while abs(tol) > 0.1
            f = @(z) 22680*C(7)^2*z^11 + 41580*C(6)*C(7)*z^10 + 36960*C(5)*C(7)*z^9 + 19250*C(6)^2*z^9 + 34650*C(5)*C(6)*z^8 + 31185*C(4)*C(7)*z^8 + 29700*C(4)*C(6)*z^7 + 23760*C(3)*C(7)*z^7 + 15840*C(5)^2*z^7 + 27720*C(4)*C(5)*z^6 + 23100*C(3)*C(6)*z^6 + 13860*C(2)*C(7)*z^6 + 22176*C(3)*C(5)*z^5 + 13860*C(2)*C(6)*z^5 + 12474*C(4)^2*z^5 + 20790*C(3)*C(4)*z^4 + 13860*C(2)*C(5)*z^4 + 13860*C(2)*C(4)*z^3 + 9240*C(3)^2*z^3 + 13860*C(2)*C(3)*z^2 + 6930*C(2)^2*z + 6930*z - 41580*a^10*C(6)*C(7) - 36960*a^9*C(5)*C(7) - 34650*a^8*C(5)*C(6) - 31185*a^8*C(4)*C(7) - 29700*a^7*C(4)*C(6) - 27720*a^6*C(4)*C(5) - 23760*a^7*C(3)*C(7) - 23100*a^6*C(3)*C(6) - 22176*a^5*C(3)*C(5) - 20790*a^4*C(3)*C(4) - 13860*a^6*C(2)*C(7) - 13860*a^5*C(2)*C(6) - 13860*a^4*C(2)*C(5) - 13860*a^3*C(2)*C(4) - 13860*a^2*C(2)*C(3) - 22680*a^11*C(7)^2 - 19250*a^9*C(6)^2 - 15840*a^7*C(5)^2 - 12474*a^5*C(4)^2 - 9240*a^3*C(3)^2 - 6930*a*C(2)^2 - 13860*s/2 - 6930*a;
            zs(2) = zs(1)-(f(zs(1))/f_dot(zs(1)));
            tol = zs(2)-zs(1);
            zs(1) = zs(2);
        end
        
        aimpoint(1,1) = zs(1);
        aimpoint(1,2) = C(1)+C(2)*aimpoint(1,1)+C(3)*aimpoint(1,1)^2+C(4)*aimpoint(1,1)^3+C(5)*aimpoint(1,1)^4+C(6)*aimpoint(1,1)^5+C(7)*aimpoint(1,1)^6;
        
    end
end