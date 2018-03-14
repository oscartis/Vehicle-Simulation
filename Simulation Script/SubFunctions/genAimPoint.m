function [headingRequest, localPath, aimPoint]= genAimPoint(X,x,trackPath)

u = x(1);

aimDist = 5;

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

d = k*norm(cP_onTrack-trackPath(cPi,:));


[Pa, aimPoint] = getPoint(d,aimDist,trackPath,cPi,Psi,pos);

headingRequest = atan2(Pa(2),Pa(1));


[~,~,i20m] = getPoint(d,50,trackPath,cPi,Psi,pos);
localPath = [cos(Psi) sin(Psi);-sin(Psi) cos(Psi)]*(trackPath(cPi:cPi+i20m,1:2)-pos)';

    function [localPoint, globalPoint,i] = getPoint(d,dist,trackPath,cPi,Psi,pos)
        
        dd =0;
        i = 0;
        
        while d < dist
            if cPi+i+1 < length(trackPath)
                dd = norm(trackPath(cPi+i+1,:) - trackPath(cPi+i,:));
                d = d + dd;
                i = i+1;
            else
                d = dist;
                i = i-1;
            end
        end
        
        h = (d - dist)/dd;
        
        R = [cos(Psi) sin(Psi);
            -sin(Psi) cos(Psi)];
        
        globalPoint = (1-h)*trackPath(cPi+i,:) + h*trackPath(cPi+i-1,:);
        localPoint = R*((globalPoint-pos)');
    end
end
