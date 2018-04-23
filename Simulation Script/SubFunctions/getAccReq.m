function [accelerationRequest,DMVelocity] = getAccReq(currentVelocity, velocityLimit, lateralAccelerationLimit,accelerationLimit, decelerationLimit, headingRequest, headingErrorDependency, localPath,x)
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
% triRes = 50;
% curveRadius = zeros(1,length(localPath)-(triRes+1)*2);
% for k = (triRes+1):length(localPath)-(triRes+1)
%     % Choose three points and make a triangle with sides
%     % A(p1p2),B(p2p3),C(p1p3)
%     A = norm(localPath(k,:)-localPath(k-4,:));
%     B = norm(localPath(k,:)-localPath(k+4,:));
%     C = norm(localPath(k-4,:)-localPath(k+4,:));
%     % Calculate triangle area
%     S = (A+B+C)/2; % Heron's formula
%     triangleArea = sqrt(S*(S-A)*(S-B)*(S-C));
%     
%     % Infinate radius check - Check if needed in C++! (not needed in MatLab that can handle inf)
%     if triangleArea == 0
%         curveRadius(k-(triRes)) = 5000; % Large value
%     else
%         % Calculate the radius of the circle that matches the points
%         curveRadius(k-(triRes)) = (A*B*C)/(4*triangleArea);
%     end
% end
% 
% % Set velocity candidate based on expected lateral acceleration
% %(remember to add safe zone)
% velocityCandidate = min(sqrt(lateralAccelerationLimit*curveRadius),velocityLimit);
% 
% % Back propagate the whole path and lower velocities if deceleration cannot
% % be achieved.
% accOK = zeros(length(velocityCandidate)-1);
% 
% while all(~accOK)
%     for k = length(velocityCandidate):-1:2
%         while ~accOK(k-1)
%             % Distance between considered path points
%             pointDistance = norm(localPath(k+1,:)-localPath(k,:));
%             % Time between points if using averaged velocity
%             timeBetweenPoints = pointDistance/((velocityCandidate(k)+velocityCandidate(k-1))/2);
%             % Requiered acceleration to achieve velocity of following point from previous point
%             requieredAcceleration = (velocityCandidate(k)-velocityCandidate(k-1))/timeBetweenPoints;
%             
%             
%             % If acceleration can't be achieved
%             if requieredAcceleration < decelerationLimit || requieredAcceleration > accelerationLimit
%                 % Set acceleration to maxBrakingAcceleration
%                 if requieredAcceleration < 0
%                     % Set acceleration to maxBrakingAcceleration
%                     modifiedDeceleration = decelerationLimit;
%                     velocityCandidate(k-1) = velocityCandidate(k)-modifiedDeceleration*timeBetweenPoints; % time based on average(v2-v1)/2
%                     
%                 elseif requieredAcceleration > 0
%                     % Set acceleration to maxAcceleration
%                     modifiedDeceleration = accelerationLimit;
%                     velocityCandidate(k) = velocityCandidate(k-1)+modifiedDeceleration*timeBetweenPoints; % time based on average(v2-v1)/2
%                 else
%                     % This case shouldn't occur, only here as fail safe.
%                 end
%                % timeBetweenPoints = pointDistance/((velocityCandidate(k)+velocityCandidate(k-1))/2);
%                 requieredAcceleration = (velocityCandidate(k)-velocityCandidate(k-1))/(2*pointDistance);
%                 % Calculate new required acceleration
%             end
%             
%             if sqrt(requieredAcceleration^2+(velocityCandidate(k)^2/curveRadius(k))^2) > lateralAccelerationLimit + 1e-3
%                 [~,imax] = max(abs(velocityCandidate(k-1:k)));
%                 velocityCandidate(k-1:k) = velocityCandidate(k+(1-imax));
%             else
%                 accOK(k-1) = true;
%             end
%         end
%     end
% end
% 
% % Choose velocity to achieve
% velocityAimpoint = 2; %On which path point to base acceleration
% % Limit it dependent on the heading request (heading error)
% desiredVelocity = velocityCandidate(velocityAimpoint-1)/(1 + headingErrorDependency*abs(headingRequest));
% 
% % Calculate time to desired velocity
% distanceToVelocity = norm(localPath(velocityAimpoint,:));
% 
% % Transform into acceleration
% accelerationRequest = (desiredVelocity-currentVelocity)/(2*distanceToVelocity);
% 
% % Limit acceleration request for positive acceleration
% if accelerationRequest > 0 && accelerationRequest > accelerationLimit
%     accelerationRequest = accelerationLimit;
%     accelerationLimited = 1;
% else
%     accelerationLimited = 0;
% end
% 
% % Limit acceleration request for negative acceleration
% if accelerationRequest < 0 && accelerationRequest < decelerationLimit
%     accelerationRequest = decelerationLimit;
%     decelerationLimited = 1;
% else
%     decelerationLimited = 0;
% end
% DMVelocity.accelerationRequest = accelerationRequest;
% DMVelocity.desiredVelocity = desiredVelocity;
% DMVelocity.velocityCandidate = velocityCandidate;
% DMVelocity.velocityAimpoint = velocityAimpoint;
% DMVelocity.accelerationLimited = accelerationLimited;
% DMVelocity.decelerationLimited = decelerationLimited;
% DMVelocity.curveRadius = curveRadius;

accelerationRequest = (10-x(1));
end