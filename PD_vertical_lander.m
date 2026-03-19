clearvars; clc;

%define our constants
m = 1000;
g = 1.62; %m/s^2
maxThrust = 2160; %Newtons

%time governance
dt = 0.01; %seconds
T = 100; %seconds

%initial state declaration
z=100; %height
v=-10; %m/s

%minimum kinematic deceleration
if maxThrust-m*g < m*(v^.2)/(2*z)
    fprintf("Minimum MaxThrust to not crash at this intitial altitude is: %.2f N\n", m*(v^.2)/(2*z))
    return
end

%calculate optimal speed of response (Kp) and damping (Kd) 
%satisfy conditions that z(tf) = v(tf) = 0
%Kp controller; 'spring stiffness'. Agressiveness with which the lander adjusts
wn = -1*((z+v*T)/(z*T));
Kp = m*wn^.2;
Kd = 2*sqrt(Kp*m);


%storage arrays
%allows you to plot graphs, analyze behavior
time = 0:dt:T; %make an array the size of T/dt representing every time instance (x-axis)
Z = zeros(size(time)); %log height for each dt
V = zeros(size(time)); %log velocity for each dt
TH = zeros(size(time)); %log thrust TH for each dt

%simulation loop
for k = 1:length(time)
    %PD controller
    if z>0
        if v < -0.5
            thrust = m*g + Kp*z - Kd*(v); %we want our base thrust to be mg, and less if we are below our desired value (0)
        else
            thrust = m*g;
        end
        %range thrust
        thrust = max(0,min(thrust, maxThrust)); %if the sim calls for negative thrust, thrust=0. Thrusters dont go the other way lol.
        %if controller calls for thrust>maxThrust, thrust=maxThrust. No turbo.
        
        %physics
        Fnet = thrust - m*g; %thrust is pos and mg always down
        a = Fnet/m;
    
        % Update state with Euler Integration. 
        v = v+a*dt;
        z = z+v*dt; % z =z+v*dt+0.5*a*dt.^2 works for CONSTANT ACCELERATION
        %not necessarily dynamic systems w changing accel.
        
        %store functions
        Z(k) = z; % log height
        V(k) = v; % log velocity
        TH(k) = thrust; % log thrust
    elseif z<=0
        z=0;
        v=0;
        thrust = 0;
        Z(k) = z; % log height
        V(k) = v; % log velocity
        TH(k) = thrust; % log thrust
        break;
    end
end

% points of interest
% Calculate the time of landing based on the height reaching zero
tidx = find(Z <= 0, 1, 'first'); % first index 'dt' within time fn at which Z<0. NOT AN ACTUAL TIME
t_landing = time(tidx); %this calls time array at that index
% Calculate the landing time and display it
fprintf('Landing time: %.2f seconds\n', t_landing);

fprintf('Height: %.2f m\n', Z(1));
fprintf('Velocity at landing: %.2f m/s\n', V(tidx));
fprintf('Thrust at landing: %.2f N\n', TH(tidx));
fprintf('Controller Coeff (Kp): %.2f N/m\n', Kp); % higher controller, more jerky
fprintf('Damping Coeff (Kd): %.2f Ns/m\n', Kd); % higher coeff, less jerky

%plotting!
figure
subplot(3,1,1)
plot(time,Z)
ylabel("Height")

subplot(3,1,2)
plot(time,V)
ylabel("Velocity")

subplot(3,1,3)
plot(time,TH)
ylabel("Thrust")
xlabel("Time (s)")


