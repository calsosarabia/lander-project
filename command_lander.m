clearvars; clc;
%% Variable Definition
%function [time, Z, V, TH] = simulate_lander(controller, params, p_controller)
%initial state declaration
z=52; %height
v=-28; %m/s
z_ground = 0;
z_target = z_ground;
v_target = -1;

% time governance
dt = 0.01; %seconds
T = 600; %seconds

%% Simulation Assignment
% define simulation parameters
params = struct('z', z, 'z_target', z_target, 'z_ground', z_ground, 'v', v, 'v_target', v_target, 'T', T, 'dt', dt);
% define universal constants % energy specific controller % PD specific controller
p_controller = struct('m', 200, 'g', 1.62, 'maxThrust', 2000, 'Isp', 50, 'zeta', 1, 'omega', 0.1);


%calculate optimal speed of response (Kp) and damping (Kd) 
%Kp controller; 'spring stiffness'. Agressiveness with which the lander adjusts
% CONTROLLER SELECTION

%controllerFcn = @pd_controller;
%controllerFcn = @energy_controller;

%% PD controller results
PDC_results = simulate_lander(@pd_controller, params, p_controller);
time = PDC_results.time;
PDC_t_landing = time(find(PDC_results.Z <= 0, 1, 'first')); %this calls time array at that index
PDC_termv = PDC_results.V(find(PDC_results.Z <= 0, 1, 'first')-1); %this calls time array at that index
PDC_t_thrustON = time(find(abs(PDC_results.TH) >= 0, 1, 'first')); %this calls time array at that index


%% EC controller results
EC_results = simulate_lander(@energy_controller, params, p_controller);
EC_t_landing = time(find(EC_results.Z <= 0, 1, 'first')); %this calls time array at that index
EC_termv = EC_results.V(find(EC_results.Z <= 0, 1, 'first')-1); %this calls time array at that index
EC_t_thrustON = time(find(abs(EC_results.TH) >= 0, 1, 'first')); %this calls time array at that index


%% Bang controller results
BangC_results = simulate_lander(@bang_controller, params, p_controller);
%cgpt_writeArraysToTxt('data.txt', BangC_results)
BangC_t_landing = time(find(BangC_results.Z <= 0, 1, 'first')); %this calls time array at that index
BangC_termv = BangC_results.V(find(BangC_results.Z <= 0, 1, 'first')-1); %this calls time array at that index
BangC_t_thrustON = time(find(abs(BangC_results.TH) > 0, 1, 'first')); %this calls time array at that index

%{ 
    % points of interest
        % Calculate the time of landing based on the height reaching zero
        tidx = find(Z <= 0, 1, 'first'); % first index 'dt' within time fn at which Z<0. NOT AN ACTUAL TIME
        t_landing = time(tidx); %this calls time array at that index
    % Calculate the landing time and display it
        fprintf('Landing time: %.2f seconds\n', t_landing);
        
        fprintf('Height: %.2f m\n', Z(1));
        fprintf('Velocity at landing: %.2f m/s\n', V(tidx));
        fprintf('Thrust at landing: %.2f N\n', TH(tidx));
        fprintf('Controller Coeff (Kp): %.2f N/m\n', p.m*(p.omega)^2); % higher controller, more jerky
        fprintf('Damping Coeff (Kd): %.2f Ns/m\n', 2*p.zeta*sqrt((p.m*(p.omega)^2)*p.m)); % higher coeff, less jerky
        display(controllerFcn)
%}

%% Plot Proportional Derivative Controller Results
x_maxlim = 25;
figure
sgtitle({'Lander Simulation','Proportional Derivative Controller v. Energy Controller v. Bang-Bang Controller'})

subplot(4,1,1)
    % plot result arrays
    plot(time, PDC_results.Z,'r--'); hold on
    plot(time, EC_results.Z,'b'); hold on
    plot(time, BangC_results.Z,'black-'); hold on
    
    %plot(EC_t_landing, 0,'b.', 'MarkerSize',10); hold on
    %plot(BangC_t_landing, 0,'black.', 'MarkerSize',10)

    ylabel("Height (m)")
    legend("PD Controller","Energy Controller", "Bang-Bang Controller")
    
    %change axis ticklengths
    ax = gca;  % get current axes
    ax.TickLength = [0.005 0.01];% minor major ticks lengths
    xlim([0,x_maxlim]);

    %% h_EC - h_PDC
    %{
subplot(4,1,2)
    plot(time, PDC_results.Z - EC_results.Z,'black:','LineWidth',2); hold on
    ylabel("h_{PD} - h_{EC} (m)")
    legend('PDC Height\newlineminus EC Height')
    
    %change axis ticklengths
    ax = gca;  % get current axes
    ax.TickLength = [0.005 0.01];% minor major ticks lengths

    xlim([0,x_maxlim]);
%}
%% Velocity Graph
subplot(4,1,2)
    plot(time,PDC_results.V,'r--','LineWidth',1); hold on
    plot(time, EC_results.V,'b','LineWidth',1); hold on
    plot(time, BangC_results.V,'black-','LineWidth',1); hold on
    
    ylabel("Velocity (m/s)")
    legend("PDC","EC","BangC")
       
    %change axis ticklengths
    ax = gca;  % get current axes
    ax.TickLength = [0.005 0.01];% minor major ticks lengths

    xlim([0,x_maxlim]);
    ylim([-25,1]);
%% Thrust Graph
subplot(4,1,3)
    plot(time,PDC_results.TH,'r--'); hold on
    plot(time, EC_results.TH,'b'); hold on
    plot(time, medfilt1(BangC_results.TH,3),'black-')
    ylabel("Thrust (N)")
    
    legend("PDC","EC", "BangC")
    
    %change axis ticklengths
    ax = gca;  % get current axes
    ax.TickLength = [0.005 0.01];% minor major ticks lengths
    
    xlim([0,x_maxlim]);
    ylim([0,2500]);
    xlabel('Time (s)')

subplot(4,1,4)
    plot(time,PDC_results.M,'r--'); hold on
    plot(time, EC_results.M,'b'); hold on
    plot(time, BangC_results.M,'black-')
    ylabel("Mass (kg)")
    xlabel('Time (s)')
    legend("PDC","EC","BangC")

        %change axis ticklengths
    ax = gca;  % get current axes
    ax.TickLength = [0.005 0.01];% minor major ticks lengths

    xlim([0,x_maxlim]);
    ylim([0,220]);

    xlabel('Time (s)')
    ylabel("Mass of Lander (kg)")

    MassData = sprintf("PDC Fuel Used (kg): %.2f kg, ", p_controller.m - PDC_results.M(find(PDC_results.Z <= 0, 1, 'first')))+ ...
    sprintf("EC Fuel Used (kg): %.2f kg, ", p_controller.m - EC_results.M(find(EC_results.Z <= 0, 1, 'first')))+...
    sprintf("BangC Fuel Used (kg): %.2f kg\n", p_controller.m - BangC_results.M(find(BangC_results.Z <= 0, 1, 'first')));

    text(0,-100,0,MassData)
    text(0,-120,0,structFieldStr(p_controller) + "" + structFieldStr(params))
    hello = structFieldStr(p_controller) + "" + structFieldStr(params)
    
Value = {'t_xtarget (s)';'vfinal (s)';'t_ThrustON (s)'};
PDC = [PDC_t_landing ; PDC_termv ; PDC_t_thrustON];
BangC = [BangC_t_landing ; BangC_termv ; BangC_t_thrustON];
EC = [EC_t_landing ; EC_termv ; EC_t_thrustON ];
    

Controller = {'PDC';'BangC';'EC'};
t_xtarget = [PDC_t_landing ; BangC_t_landing ; EC_t_landing];
vtarget = [PDC_termv; BangC_termv; EC_termv];
t_ThrustON = [PDC_t_thrustON; BangC_t_thrustON; EC_t_thrustON];
%{
Controller = {'PDC';'EC'};
t_xtarget = [PDC_t_landing ; EC_t_landing];
t_vtarget = [PDC_termv; EC_termv];
t_ThrustON = [PDC_t_thrustON; EC_t_thrustON];
%}

Table = table(Controller,t_xtarget,vtarget,t_ThrustON);
% Display the results in a table format
disp(Table);





