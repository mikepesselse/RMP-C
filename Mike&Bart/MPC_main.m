%% MPC Assignment
% Mike Pesselse and Bart de Jong
clc
clearvars
close all force
yalmip('clear')

folder = fileparts(which(mfilename)); 
addpath(genpath(folder));

%% Initialisation
Ts = 0.01;
             
mpc_sim.Q   =   [1e4   0   0   0   0   0;
                 0     1e4 0   0   0   0;
                 0     0   1e4 0   0   0;
                 0     0   0   1e4 0   0;
                 0     0   0   0   1e4 0;
                 0     0   0   0   0   1e4];         
             
mpc_sim.Qt  = mpc_sim.Q*1e3;     % Terminal state matrix
mpc_sim.R   = eye(3)*0.1;        % Input matrix
mpc_sim.W   = mpc_sim.Q;         % Waypoint matrix
mpc_sim.N   = ceil(1/Ts);        % Control horizon

%% Simulation settings
x0 = [0.1 0 1 0 0 0]';
wayp1 = [0 0 0.5 -1 0.2 0];
wayp2 = [0 0 0.4 -1 0.2 0];
wayp3 = [0 0 0.5 -1 -0.2 0];
wayp4 = [0 0 0.4 -1 -0.2 0];
simT = 1.5;              % Simulation time [s]
ulim = [1 1 1]*1;              % Maximum absolute value input

%% Run simulations
[results_mpc] = MPC_Controller2(mpc_sim, x0, simT, Ts, ulim, wayp1, wayp2, wayp3, wayp4);

%% Plot results
results = results_mpc;

figure; plot3(results.state(:, 1), results.state(:, 3), results.state(:, 5));
hold on
plot3(wayp1(1), wayp1(3), wayp1(5), 'r*')
plot3(wayp2(1), wayp2(3), wayp2(5), 'r*')
plot3(wayp3(1), wayp3(3), wayp3(5), 'r*')
plot3(wayp4(1), wayp4(3), wayp4(5), 'r*')
plot3(0, 0, 0, 'b*')
plot3(x0(1), x0(3), x0(5), 'k*')
xlabel('x'); ylabel('y'); zlabel('z');
legend('Trajectory', 'waypoint 1', 'waypoint 2', 'waypoint 3', 'waypoint 4', 'goal', 'start')
grid on
axis equal

