function [accelerationRequest,DMVelocity] = getAccReq(currentVelocity, velocityLimit, lateralAccelerationLimit,accelerationLimit, decelerationLimit, headingRequest, headingErrorDependency, localPath,curveRadius)
%driverModelVelocity_v2 calculates the desired acceleration of the CFSD18
%vehicle.
%
%Input
%   CURRENTVELOCITY             [1 x 1] Velocity of the vehicle [m/s]
%   VELOCITYLIMIT               [1 x 1] Maximum allowed velocity [m/s]
%   LATERALACCELERATIONLIMIT    [1 x 1] Allowed lateral acceleration [m/s^2]
%   ACCELERATIONLIMIT           [1 x 1] Allowed longitudinal acceleration (positive) [m/s^2]
%   DECELERATIONLIMIT           [1 x 1] Allowed longitudinal acceleration (negative) [m/s^2]
%   HEADINGREQUEST              [1 x 1] Heading error to aimpoint [rad]
%   HEADINGERRORDEPENDENCY      [1 x 1] Constant
%   PATH                        [n x 2] Local coordinates of the path [x,y]
%
%Output
%   ACCELERATIONREQUEST         [1 x 1] Desired acceleration
%%
% Segmentize the path and calculate radius of segments
% - First radius is calculated at 2nd path point
% - Last radius is calculated at 2nd to last path point
% Note! 3 points in a row gives infinate radius.


% Set velocity candidate based on expected lateral acceleration
%(remember to add safe zone)
velocityCandidate = min(sqrt(lateralAccelerationLimit*curveRadius),velocityLimit);

% Back propagate the whole path and lower velocities if deceleration cannot
% be achieved.
accOK = zeros(length(velocityCandidate)-1);

while all(~accOK)
    for k = length(velocityCandidate):-1:2
        while ~accOK(k-1)
            % Distance between considered path points
            pointDistance = norm(localPath(k,:)-localPath(k-1,:));
            % Time between points if using averaged velocity
            timeBetweenPoints = pointDistance/((velocityCandidate(k)+velocityCandidate(k-1))/2);
            % Requiered acceleration to achieve velocity of following point from previous point
            requieredAcceleration = (velocityCandidate(k)-velocityCandidate(k-1))/timeBetweenPoints;
            
            
            % If acceleration can't be achieved
            if requieredAcceleration < decelerationLimit || requieredAcceleration > accelerationLimit
                % Set acceleration to maxBrakingAcceleration
                if requieredAcceleration < 0
                    % Set acceleration to maxBrakingAcceleration
                    modifiedDeceleration = decelerationLimit;
                    velocityCandidate(k-1) = velocityCandidate(k)-modifiedDeceleration*timeBetweenPoints; % time based on average(v2-v1)/2
                    
                elseif requieredAcceleration > 0
                    % Set acceleration to maxAcceleration
                    modifiedDeceleration = accelerationLimit;
                    velocityCandidate(k) = velocityCandidate(k-1)+modifiedDeceleration*timeBetweenPoints; % time based on average(v2-v1)/2
                else
                    % This case shouldn't occur, only here as fail safe.
                end
                timeBetweenPoints = pointDistance/((velocityCandidate(k)+velocityCandidate(k-1))/2);
                requieredAcceleration = (velocityCandidate(k)-velocityCandidate(k-1))/timeBetweenPoints;
                % Calculate new required acceleration
            end
            
            if sqrt(requieredAcceleration^2+(velocityCandidate(k)^2/curveRadius(k))^2) > lateralAccelerationLimit + 1e-3
                [~,imax] = max(abs(velocityCandidate(k-1:k)));
                velocityCandidate(k-1:k) = velocityCandidate(k+(1-imax));
            else
                accOK(k-1) = true;
            end
        end
    end
end

% Choose velocity to achieve
velocityAimpoint = 2; %On which path point to base acceleration
% Limit it dependent on the heading request (heading error)
desiredVelocity = velocityCandidate(velocityAimpoint-1)/(1 + headingErrorDependency*abs(headingRequest));

% Calculate time to desired velocity
distanceToVelocity = norm(localPath(velocityAimpoint,:));

% Transform into acceleration
accelerationRequest = (desiredVelocity-currentVelocity)/(2*distanceToVelocity);

% Limit acceleration request for positive acceleration
if accelerationRequest > 0 && accelerationRequest > accelerationLimit
    accelerationRequest = accelerationLimit;
    accelerationLimited = 1;
else
    accelerationLimited = 0;
end

% Limit acceleration request for negative acceleration
if accelerationRequest < 0 && accelerationRequest < decelerationLimit
    accelerationRequest = decelerationLimit;
    decelerationLimited = 1;
else
    decelerationLimited = 0;
end
DMVelocity.accelerationRequest = accelerationRequest;
DMVelocity.desiredVelocity = desiredVelocity;
DMVelocity.velocityCandidate = velocityCandidate;
DMVelocity.velocityAimpoint = velocityAimpoint;
DMVelocity.accelerationLimited = accelerationLimited;
DMVelocity.decelerationLimited = decelerationLimited;
DMVelocity.curveRadius = curveRadius;


end