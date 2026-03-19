function thrust = energy_controller(z, v, z_target, v_target, p)
% if alredy on or below the ground, shut off the thrust
if z <= 0
    thrust = 0;
    return
end
% only brake if descending
if v < 0
    %instantaneous thrust dependent on v and z.
    a_req = (v^2)/(2*z);
    thrust = p.m*(p.g+a_req);
else
    %only moving upward is left, in which case, hover thrust.
    thrust = p.m*p.g;
end

%saturate the thrust within physical bounds, no negative thrust
thrust = max(0, min(thrust, p.maxThrust));
end