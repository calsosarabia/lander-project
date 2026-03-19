function thrust = pd_controller(z, v , z_target, v_target, p) %p.m, p.g, p.maxthrust, p.zeta (damping), p.omega
%pd controllers with p.zeta < 1 will oscillate around z_target.
%pd controller landers MUST be critically damped.
% also -- v_target is -0.5 m/s to avoid mathematically impossible 'hover'
% at v= 0 m/s.

% only brake if descending
if nargin < 5
    thrust = 0.1;
else
    delta_v = v - v_target;
    delta_z = z - z_target;
    Kp = (p.omega)^.2; %Calculate proportional coefficient, 1/s^2
    Kd = 2 * p.zeta * p.omega; % Calculate the damping coefficient, 1/s
    %coordinate definition implies getting closer to target as negative
    %(downwards)
    thrust = p.m*(p.g + Kp*(z-z_target) + Kd*(abs(v-v_target)));
    end
    % if alredy on or below the ground, shut off the thrust
    if delta_z <= 0
        thrust = 0;
        return
    end

    %saturate the thrust
    thrust = max(0, min(thrust, p.maxThrust));
end