function [results_mpc] = MPC_Controller(DTSS1, mpc_sim, x0, simT, Ts, ulim, xlim)
m = 0.030;  % weight (in kg) with 5 vicon markers (each is about 0.25g)
g = 9.81;   % gravitational constant
I = [1.43e-5,   0,          0; % inertial tensor in m^2 kg
    0,         1.43e-5,    0;
    0,         0,          2.89e-5];
L = 0.046; % arm length in m

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.grav = g;
params.arm_length = L;

A = DTSS1.A;
B = DTSS1.B;

C = DTSS1.C;
D = DTSS1.D;

Q = mpc_sim.Q;
Qt = mpc_sim.Qt;
R = mpc_sim.R;
N = mpc_sim.N;

nx = size(A, 1); % Number of states
nu = size(B, 2); % Number of inputs


u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));

constraints = [];
objective = 0;
for k = 1:N
    objective = objective + norm(Q*x{k},1) + norm(R*u{k},1);
    if k == N
        objective = objective + norm(Qt*x{k}, 1);
    end
    constraints = [constraints, x{k+1} == A*x{k} + B*u{k}];
    %     constraints = [constraints, -ulim <= u{k}<= ulim];
    %     constraints = [constraints, -xlim<=x{k+1}(3)<=xlim];
end
controller = optimizer(constraints, objective,[],x{1},[u{:,1}]);

x_true = x0;
implementedU = [];
xn = x_true;

f = waitbar(0,'1','Name','Simulating MPC controller',...
    'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');
for i = 1:simT/Ts

    U = controller{xn};
    
    %     [~,x_true] = ode45(@(t,xs) pendulumdynamics(xs, U(1), g, l, b_a, b_p, m, M), [0 Ts], x_true);
    %       [~, x_true] = ode45(@(t,s) quadEOM(t, s, U(:,1), params), [0 Ts], x_true);
    x_true = A*x_true + B*U(:,1);
    
    %     x_true = x_true(end, :)';
    %     x_true(1) = wrapToPi(x_true(1));
    
    
    y = C*x_true;
    xn = x_true;
    
    
    % Save control inputs and state
    implementedU = [implementedU;U(1)];
    state(i, :) = x_true';
    
    waitbar(i/(simT/Ts),f,sprintf(num2str(ceil(i/(simT/Ts)*100))))
    
end

results_mpc.U       = implementedU;
results_mpc.state   = state;

results_mpc.title = "MPC controller without noise and observer";

delete(f)
end

