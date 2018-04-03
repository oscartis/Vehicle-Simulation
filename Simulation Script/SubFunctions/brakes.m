function Fx = brakes(accelerationRequest,m)

if accelerationRequest > 0
    Fx = [0;0;0;0];
    return
end

brakeDist = [.25; .25; .75; .75]/2;
Fx = accelerationRequest*m*brakeDist;
end