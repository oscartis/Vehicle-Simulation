function [localPath] = curveEstimation(localPath)
DoP = 80;
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
end