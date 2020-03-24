%% Robot Motion Planning and Control Assignment Q3 2019/2020
% Daan Bruggink     [.......]
% Bart de Jong      [4367146]
% Tom Kerssemakers  [.......]
% Mike Pesselse     [.......]
clc
clearvars
close all force
yalmip('clear')

%% Initialisation
Ts = 0.01;
             
mpc_sim.Q   =   [1e4   0   0   0   0   0;
                 0     1e4 0   0   0   0;
                 0     0   1e4 0   0   0;
                 0     0   0   1e4 0   0;
                 0     0   0   0   1e4 0;
                 0     0   0   0   0   1e4];                    
mpc_sim.Qt  = mpc_sim.Q*1e3;    % Terminal state matrix
mpc_sim.R   = eye(3)*1e-2;      % Input matrix
mpc_sim.W   = mpc_sim.Q;        % Waypoint matrix
mpc_sim.N   = ceil(1/Ts);       % Control horizon

%% Simulation settings
x0 = [0 0 0 0 0 0]';            % Define start position
ring1 = [ 1 0  3  2  1  0];     % [x, desired x-velocity, y, desired y-velocity, z, desired z-velocity]
ring2 = [-1 0  6  2 -1  0];     % [x, desired x-velocity, y, desired y-velocity, z, desired z-velocity]
goal = [0 0 10 0 0 0];          % Define goal position
simT = 5;                       % Simulation time [s]
ulim = [1 1 1]*7;               % Maximum absolute value input [x y z]

%% Run simulations
[results] = MPC_Controller2(mpc_sim, x0, simT, Ts, ulim, ring1, ring2, goal);

%% Plot results
figure; plot3(results.state(:, 1), results.state(:, 3), results.state(:, 5));
hold on
plot3(ring1(1), ring1(3), ring1(5), 'ro')
plot3(ring2(1), ring2(3), ring2(5), 'ro')
plot3(goal(1), goal(3), goal(5), 'b*')
plot3(x0(1), x0(3), x0(5), 'k*')
xlabel('x'); ylabel('y'); zlabel('z');
legend('Trajectory', 'ring 1', 'ring 2', 'goal', 'start')
grid on
xlim([-1.5 1.5]); ylim([0 10]); zlim([-1.5 1.5])
pbaspect([3 10 3])

