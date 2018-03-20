function Fx = brakes(accelerationRequest,m)

if accelerationRequest > 0
    Fx = [0;0;0;0];
    return
end

brakeDist = [.75; .75; .25; .25]/2;
Fx = accelerationRequest*m*brakeDist;
end