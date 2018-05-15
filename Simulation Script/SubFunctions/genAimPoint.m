function [headingRequest, localPath, aimPoint, d, cPi, cP_onTrack,curveRadius]= genAimPoint(X,x,trackPath)

u = x(1);

localDist = 30;
aimDist = 8;

pos = X(1:2)';
Psi = X(3);

dist = zeros(length(trackPath),1);

for i = 1:length(trackPath)
    dist(i) = norm((trackPath(i,:)-pos));
end

[~,cPi] = min(abs(dist));
cP = trackPath(cPi,:);

carVec = pos-cP;
if norm(carVec) ~= 0
    carVecNorm = carVec/norm(carVec);
else
    carVecNorm = carVec;
end

nextVec = trackPath(cPi+1,:)-cP;
nextVecNorm = nextVec/norm(nextVec);

if cPi-1 > 1
    prevVec = trackPath(cPi-1,:)-cP;
else
    prevVec = trackPath(1,:)-cP;
end

if norm(prevVec) ~= 0
    prevVecNorm = prevVec/norm(prevVec);
else
    prevVecNorm = prevVec;
end

alpha1 = acos(dot(nextVecNorm,carVecNorm));
alpha2 = acos(dot(prevVecNorm,carVecNorm));

if alpha1 >= pi/2 && alpha2 >= pi/2 || norm(carVecNorm) == 0
    cP_onTrack = cP;
    k = 0;
    
elseif alpha1 < pi/2 && alpha1 < alpha2
    cP_onTrack = cP + nextVec*dot(carVec,nextVec);
    k = -1;
elseif alpha2 < pi/2 && alpha2 < alpha1
    cP_onTrack = cP + prevVec*dot(carVec,prevVec);
    k = 1;
else
    disp('we fugd up')
    cP_onTrack = [0,0];
    k = 0;
end

trackCopy = trackPath;
p2carDist = 0;
trackCarFrame = [cos(-Psi), -sin(-Psi);sin(-Psi), cos(-Psi)]*(trackPath-pos)';
angle2point = atan2(trackCarFrame(2,:),trackCarFrame(1,:));
idxBehind = find(angle2point>pi/3 | angle2point < -pi/3);
trackCopy(idxBehind,:) = [];
dist = [];
%idxFarAway = find(norm(trackPath-pos) > 20);
for i = 1:length(trackCopy)
    dist(i) = norm(trackCopy(i,:)-pos);
    pdist(i) = (norm(trackCopy(i,:)-cP_onTrack));
end
trackCopy(dist>20,:) =[];
pdist(dist>20) = [];
[~,pidx] = min(pdist);
localPath = [];

while p2carDist < localDist
    tmpPoint = trackCopy(pidx,:);
    trackCopy(pidx,:) = [];
    pdist(pidx) = [];
    tmpCopy = trackCopy-tmpPoint;
    tmpDist = sqrt(tmpCopy(:,1).^2+tmpCopy(:,2).^2);
    
    [~,pidx] = min(tmpDist);
    p2carDist = p2carDist + min(tmpDist);
    localPath = [localPath;tmpPoint];
    
    if min(tmpDist) > 2
        break;
    end
end


d = k*norm(cP_onTrack-trackPath(cPi,:));


localPath = [cos(-Psi) -sin(-Psi);sin(-Psi) cos(-Psi)]*(localPath-pos)';
localPath(:,diff(localPath(1,:)) < 0) = [];
localPath(:,end) = [];


[localPath, aimPoint, curveRadius] = curveEstimation(localPath',aimDist);

headingRequest = atan2(aimPoint(2),aimPoint(1));

end
