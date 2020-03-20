%% MPC Assignment
% Mike Pesselse and Bart de Jong
clc
clearvars
close all
yalmip('clear')

folder = fileparts(which(mfilename)); 
addpath(genpath(folder));

%% Construct SS-models
Ts = 0.02;
% 
linstate = [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]*1e-10;     % Linearise around linstate
[CTSS1, DTSS1] = constructmodel(Ts, linstate);


%% MPC data
mpc_sim.Q   = eye(13);
mpc_sim.Q(1, 1) = 1000;
mpc_sim.Q(2,2) = mpc_sim.Q(1,1);
mpc_sim.Q(3,3) = mpc_sim.Q(2,2);
mpc_sim.Qt  = mpc_sim.Q*1000;     % Terminal state matrix
mpc_sim.R   = eye(4);                % Input matrix
mpc_sim.N   = ceil(1/Ts);         % Control horizon

%% Simulation settings
x0 = ones(13, 1);  % Initial conditions
simT = 5;              % Simulation time [s]
ulim = 15;              % Maximum absolute value input
plim = 1;               % Maximum absolute value positiond

%% Run simulations
% MPC with observer
[results_mpc] = MPC_Controller(DTSS1, mpc_sim, x0, simT, Ts, ulim, plim);

%% Plot results
results = results_mpc;

% Input plot
% figure; set(gcf,'units','normalized','outerposition',[0.5 0 .5 .5])
% stairs(results.U,'b'); hold on;
% plot(zeros(simT/Ts, 1));
% plot(1:simT/Ts,ulim*ones(simT/Ts,1),'--k'); 
% plot(1:simT/Ts,-ulim*ones(simT/Ts,1),'--k');
% ylim([-ulim*1.1 ulim*1.1]); grid on;
% ylabel('Input [N]'); xlabel('Timesteps'); title(['Input', results.title])
% legend('MPC without observer', 'no input')

% % Angle pendulum plot
% figure; set(gcf,'units','normalized','outerposition',[0 0 .5 .5])
% plot([results.state(:, 1)/pi*180 zeros(simT/Ts, 1)]); hold on;
% plot(1:simT/Ts,180*ones(simT/Ts,1),'--k'); 
% plot(1:simT/Ts,-180*ones(simT/Ts,1),'--k'); grid on;
% ylabel('deg'); xlabel('Timesteps'); title(['Angle pendulum', results.title])
% legend('MPC without observer', 'Reference')
% 
% % Position cart plot
% figure; set(gcf,'units','normalized','outerposition',[0.5 0 .5 .5])
% plot([results.state(:, 3) zeros(simT/Ts, 1)]); hold on;
% plot(1:simT/Ts,plim*ones(simT/Ts,1),'--k'); 
% plot(1:simT/Ts,-plim*ones(simT/Ts,1),'--k');
% ylim([-plim*1.1 plim*1.1]); grid on;
% ylabel('position'); xlabel('Timesteps'); title(['Position cart', results.title])
% legend('MPC without observer', 'Reference')

% % Overview plot
% txt = 'without';
% figure; set(gcf,'units','normalized','outerposition',[0.5 0.5 .5 .5])
% ax1 = subplot(2,1,1);
% plot([results_mpc_no_observer.state(:, 1)/pi*180 zeros(simT/Ts, 1) results_pid.state(:, 1)/pi*180 results_mpc.state(:, 1)/pi*180]); hold on;
% plot(1:simT/Ts,180*ones(simT/Ts,1),'--k'); 
% plot(1:simT/Ts,-180*ones(simT/Ts,1),'--k'); grid on;
% ylabel('deg'); xlabel('Timesteps'); title(['Angle pendulum ', txt, ' noise'])
% legend('MPC without observer', 'Reference', 'PID','MPC with observer')
% 
% ax2 = subplot(2,1,2); 
% plot([results_mpc_no_observer.state(:,3) zeros(simT/Ts, 1) results_pid.state(:, 3) results_mpc.state(:, 3)]); hold on;
% plot(1:simT/Ts,plim*ones(simT/Ts,1),'--k'); 
% plot(1:simT/Ts,-plim*ones(simT/Ts,1),'--k');
% ylim([-plim*1.1 plim*1.1]); grid on;
% ylabel('m'); xlabel('Timesteps'); title(['Position cart ', txt, ' noise'])
% legend('MPC without observer', 'Reference', 'PID','MPC with observer')
% linkaxes([ax1,ax2],'x');


%% Animate results
% figure
% % set(gcf, 'Units', 'Centimeters', 'Position', [-60 -60 30 15])
% set(gcf,'units','normalized','outerposition',[0 0.5 .5 .5])
% title(['Simulation', results.title])
% 
% y(:, 1) = results.state(:, 3);
% y(:, 2) = results.state(:, 4);
% y(:, 3) = results.state(:, 1);
% y(:, 4) = results.state(:, 2);
% 
% tic
% for k=1:simT/Ts
%     drawcartpend(y(k,:),2,2,model.l);
% end
% toc
