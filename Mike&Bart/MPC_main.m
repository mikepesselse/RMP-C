%% Robot Motion Planning and Control Assignment Q3 2019/2020
% Daan Bruggink     [4318064]
% Bart de Jong      [4367146]
% Tom Kerssemakers  [.......]
% Mike Pesselse     [4300564]
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

%% Ring initialisation
% Original points, original plane
t = linspace(0,2*pi);
x = cos(t);
y = sin(t);
z = 0*t;
pnts = [x;y;z];
% unit normal for original plane
n0 = [0;0;0.1]; 
n0 = n0/norm(n0);
% unit normal for plane to rotate into 
% plane is orthogonal to n1... given by equation
% n1(1)*x + n1(2)*y + n1(3)*z = 0
n1 = [0;1;0]; 
n1 = n1/norm(n1); 
% theta is the angle between normals
c = dot(n0,n1) / ( norm(n0)*norm(n1) ); % cos(theta)
s = sqrt(0.1-c*c);                        % sin(theta)
u = cross(n0,n1) / ( norm(n0)*norm(n1) ); % rotation axis...
u = u/norm(u); % ... as unit vector
C = 0.35-c;
% the rotation matrix
R = [u(1)^2*C+c, u(1)*u(2)*C-u(3)*s, u(1)*u(3)*C+u(2)*s
    u(2)*u(1)*C+u(3)*s, u(2)^2*C+c, u(2)*u(3)*C-u(1)*s
    u(3)*u(1)*C-u(2)*s, u(3)*u(2)*C+u(1)*s, u(3)^2*C+c];
% Rotated points1.5
newPnts = R*pnts;

randx1 = randi([-2 2],1);
randy1 = randi([1 4],1);
randz1 = randi([-2 2],1);

randx2 = randi([-2 2],1);
randy2 = randi([5 9],1);
randz2 = randi([-2 2],1);
%{
plot3(X,Y,Z)
hold on
plot3(0.4+0.5*newPnts(1,:),0.4+0.5*newPnts(2,:),0.75+0.5*newPnts(3,:),'LineWidth',5)
%plot3(randx+0.5*newPnts(1,:),randy+0.5*newPnts(2,:),randz+0.5*newPnts(3,:),'LineWidth',5)
xlabel('x'); ylabel('y'); zlabel('z');
grid on
%}

%% Simulation settings
x0 = [0 0 0 0 0 0]';            % Define start position
ring1 = [randx1 0  randy1  2  randz1  0];     % [x, desired x-velocity, y, desired y-velocity, z, desired z-velocity]
ring2 = [randx2 0  randy2  2 randz2  0];     % [x, desired x-velocity, y, desired y-velocity, z, desired z-velocity]
goal = [0 0 10 0 0 0];          % Define goal position
simT = 5;                       % Simulation time [s]
ulim = [1 1 1]*7;               % Maximum absolute value input [x y z]

%% Run simulations
[results] = MPC_Controller(mpc_sim, x0, simT, Ts, ulim, ring1, ring2, goal);

%% Plot results
figure; plot3(results.state(:, 1), results.state(:, 3), results.state(:, 5));
hold on
plot3(ring1(1)+1*newPnts(1,:), ring1(3)+1*newPnts(2,:), ring1(5)+1*newPnts(3,:), 'LineWidth',5)
plot3(ring2(1)+1*newPnts(1,:), ring2(3)+1*newPnts(2,:), ring2(5)+1*newPnts(3,:), 'LineWidth',5)
plot3(goal(1), goal(3), goal(5), 'b*')
plot3(x0(1), x0(3), x0(5), 'k*')
xlabel('x'); ylabel('y'); zlabel('z');
legend('Trajectory', 'ring 1', 'ring 2', 'goal', 'start')
grid on
xlim([-3 3]); ylim([0 10]); zlim([-3 3])
pbaspect([3 10 3])

