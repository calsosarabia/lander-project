function thrust = bang_controller(z, v , z_target, v_target, p) %p.m, p.g, p.maxthrust, p.zeta (damping), p.omega
%pd controllers with p.zeta < 1 will oscillate around z_target.
%pd controller landers MUST be critically damped.
% also -- v_target is -0.5 m/s to avoid mathematically impossible 'hover'
% at v= 0 m/s.

% only brake if descending
if nargin < 5
    thrust = 0;
else
    delta_v = v - v_target;
    delta_z = z - z_target;
    %% Compute critical height
    a_max = (p.maxThrust)/p.m - p.g;

    d_crit = ((delta_v)^2)/(2*a_max);
% bang bang controller because it's not a state switch  controller, it
% moves back and forth based on current values. However, because maxThrust
% caps the called thrust, the lander creates the function and part of it is
% below maxThrust but the other is supposed to be above, landing correctly.
% Moreover, the critical distance computation makes the system wait until
% full throttle saves the rocket, but pd controller doesn't always call for
% full throttle at the end, resulting in a crash.
    if delta_z > d_crit
        thrust = 0; % No thrust needed if above critical height
    else
        thrust = p.maxThrust;
    end
    % if alredy on or below the ground, shut off the thrust
    if delta_z <= 0
        thrust = 0;
        return
    end
end