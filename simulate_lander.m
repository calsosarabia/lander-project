function results = simulate_lander(controllerFcn, params, p_controller)
% define simulation parameters
%params = [z, z_target, z_ground, v, v_target, T, dt];
% define universal constants % energy specific controller % PD specific controller
%p_controller = struct('m', 1000, 'g', 1.62, 'maxThrust', 2130, 'zeta', 1, 'omega', 1.1);

% Initialize input variables
dt = params.dt;
T = params.T;

z = params.z;
z_ground = params.z_ground;
z_target = params.z_target;

v = params.v;
v_target = params.v_target;

p = p_controller;
m = p_controller.m;
g = p_controller.g;
Isp = p_controller.Isp;
% storage arrays
%allows you to plot graphs, analyze behavior
time = 0:dt:T; %make an array the size of T/dt representing every time instance (x-axis)
Z = zeros(size(time)); %log height for each dt
V = zeros(size(time)); %log velocity for each dt
TH = zeros(size(time)); %log thrust TH for each dt
M = zeros(size(time));

results = struct('time', time, 'Z', Z, 'V', V, 'TH', TH, 'M', M);

%% Simulation loop    
for k = 1:length(time)
    %controller
    if v < v_target
        thrust = controllerFcn(z, v, z_target, v_target, p);
    else
        thrust = m*g;
    end
    %physics
    Fnet = thrust - m*g; %thrust is pos and mg always down
    a = Fnet/m;

    % Update state with Euler Integration. 
    v = v+a*dt;
    z = z+v*dt; % z =z+v*dt+0.5*a*dt.^2 works for CONSTANT ACCELERATION
    %not necessarily dynamic systems w changing accel.
    
    % mass fn calculator
    mfr = thrust/(Isp*g);
    m = m - mfr * dt;

    %store functions
    results.Z(k) = z; % log height
    results.V(k) = v; % log velocity
    results.TH(k) = thrust; % log thrust
    results.M(k) = m;

    if z < z_ground
        break
    end
%{
    if v >= v_target
        k2 = abs((z/v_target)/dt)
        for h = k:k2
            thrust = m*g;
            Z(h) = z; % log height
            V(h) = v; % log velocity
            TH(h) = thrust; % log thrust
        end
        break
    end
%}
end

%trim arrays
%{
 Trim the arrays to the actual length of the simulation
Z = Z(1:(k));
V = V(1:(k));
TH = TH(1:(k));
time = time(1:(k));
%}
end