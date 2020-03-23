function [results_mpc] = MPC_Controller(mpc_sim, x0, simT, Ts, ulim, wayp1, wayp2)
m = 0.030;  % weight (in kg) with 5 vicon markers (each is about 0.25g)
g = 9.81;   % gravitational constant

Q = mpc_sim.Q;
Qt = mpc_sim.Qt;
W = mpc_sim.W;
R = mpc_sim.R;
N = mpc_sim.N;

% nx = size(A, 1); % Number of states
% nu = size(B, 2); % Number of inputs
nx = 6;
nu = 3;


u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));

% xwayp = zeros(12,1);
wayp1 = wayp1';
wayp2 = wayp2';
rho = 1e5;
kdes1 = 25;
kdes2 = 65;

constraints = [];
objective = 0;
for k = 1:N
    %     objective = objective + norm(Q*x{k},1) + norm(R*u{k},1);
    %     %         objective = objective + norm(W*(x{k}-wayp),1)*sqrt(rho/2/pi)*exp(-rho/2*(k-kdes)^2);
    %     %     if k == N
    %     %                 objective = objective + norm(Qt*x{k}, 1);
    %     %     end
    %     if k == kdes
    %         objective = objective + norm(W*(x{k}-wayp),1);
    %     end
    %     constraints = [constraints, x{k+1}(1)  == x{k}(1)  + x{k}(2)*Ts];
    %     cosntraints = [constraints, x{k+1}(2)  == x{k}(2)  + u{k}(1)/m*x{k}(7)*Ts];
    %     constraints = [constraints, x{k+1}(3)  == x{k}(3)  + x{k}(4)*Ts];
    %     constraints = [constraints, x{k+1}(4)  == x{k}(4)  + u{k}(1)/m*x{k}(9)*Ts];
    %     constraints = [constraints, x{k+1}(5)  == x{k}(5)  + x{k}(6)*Ts];
    %     constraints = [constraints, x{k+1}(6)  == x{k}(6)  + (u{k}(1)/m-g)*Ts];
    %     constraints = [constraints, x{k+1}(7)  == x{k}(7)  + x{k}(8)*Ts];
    %     constraints = [constraints, x{k+1}(8)  == x{k}(8)  + u{k}(2)*Ts];
    %     constraints = [constraints, x{k+1}(9)  == x{k}(9)  + x{k}(10)*Ts];
    %     constraints = [constraints, x{k+1}(10) == x{k}(10) + u{k}(3)*Ts];
    
    constraints = [constraints, x{k+1}(1)  == x{k}(1)  + x{k}(2)*Ts];
    cosntraints = [constraints, x{k+1}(2)  == x{k}(2)  + u{k}(1)/m*Ts];
    constraints = [constraints, x{k+1}(3)  == x{k}(3)  + x{k}(4)*Ts];
    constraints = [constraints, x{k+1}(4)  == x{k}(4)  + u{k}(2)/m*Ts];
    constraints = [constraints, x{k+1}(5)  == x{k}(5)  + x{k}(6)*Ts];
    constraints = [constraints, x{k+1}(6)  == x{k}(6)  + (u{k}(3)/m-g)*Ts];
    
    
    %     constraints = [constraints, abs(x{k}(7)) <= 15/180*pi];
    %     constraints = [constraints, abs(x{k}(9)) <= 15/180*pi];
    
    constraints = [constraints, -ulim(1) <= u{k}(1)<= ulim(1)];
    constraints = [constraints, -ulim(2) <= u{k}(2)<= ulim(2)];
    constraints = [constraints, 0 <= u{k}(3)<= ulim(3)];
    
end
% constraints = [constraints, norm([x{kdes}(1) x{kdes}(3) x{kdes}(5)]-[wayp(1) wayp(3) wayp(5)]) <= 0.05];
% options = sdpsettings('solver','bmibnb', 'debug', 1, 'verbose', 1);%, 'bmibnb.roottight',[0|1]);
% controller = optimizer(constraints, objective,options,x{1},[u{:}]);
% controller = optimizer(constraints, objective,[],x{1},[u{:}]);


x_true = x0;
implementedU = [];
xn = x_true;

f = waitbar(0,'1','Name','Simulating MPC controller',...
    'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');
for i = 1:simT/Ts
    objective = 0;
    if i < kdes1
        for k = 1:N
            objective = objective + norm(Q*x{k},1) + norm(R*u{k},1);
            objective = objective + norm(W*(x{k}-wayp1),1)*sqrt(rho/2/pi)*exp(-rho/2*(k-(kdes1-i))^2);
        end
    elseif i < kdes2
        for k = 1:N
            objective = objective + norm(Q*x{k},1) + norm(R*u{k},1);
            objective = objective + norm(W*(x{k}-wayp2),1)*sqrt(rho/2/pi)*exp(-rho/2*(k-(kdes2-i))^2);
        end
    else
        for k = 1:N
            objective = objective + norm(Q*x{k},1) + norm(R*u{k},1);
            if k == N
                objective = objective + norm(Qt*x{k}, 1);
            end
        end
    end
    options = sdpsettings('solver', 'quadprog');
    
    controller = optimizer(constraints, objective,options,x{1},[u{:}]);
    U = controller{xn};
    
%     optimize(constraints,objective,options)
%     U = value(u{1});
    
    U = U(:,1)
    
    %     x_true(1)  = x_true(1)  + x_true(2)*Ts;         % x
    %     x_true(2)  = x_true(2)  + U(1)/m*x_true(7)*Ts;  % xdot
    %     x_true(3)  = x_true(3)  + x_true(4)*Ts;         % y
    %     x_true(4)  = x_true(4)  + U(1)/m*x_true(9)*Ts;  % ydot
    %     x_true(5)  = x_true(5)  + x_true(6)*Ts;         % z
    %     x_true(6)  = x_true(6)  + (U(1)/m-g)*Ts;        % zdot
    %     x_true(7)  = x_true(7)  + x_true(8)*Ts;         % theta
    %     x_true(8)  = x_true(8)  + U(2)*Ts;              % thetadot
    %     x_true(9)  = x_true(9)  + x_true(10)*Ts;        % phi
    %     x_true(10) = x_true(10) + U(3)*Ts;              % phidot
    
    x_true(1)  = x_true(1)  + x_true(2)*Ts;         % x
    x_true(2)  = x_true(2)  + U(1)/m*Ts;  % xdot
    x_true(3)  = x_true(3)  + x_true(4)*Ts;         % y
    x_true(4)  = x_true(4)  + U(2)/m*Ts;  % ydot
    x_true(5)  = x_true(5)  + x_true(6)*Ts;         % z
    x_true(6)  = x_true(6)  + (U(3)/m-g)*Ts;        % zdot
    
    xn = x_true;
    
    % Save control inputs and state
    implementedU = [implementedU;U(:,1)'];
    state(i, :) = x_true';
    
    waitbar(i/(simT/Ts),f,sprintf(num2str(ceil(i/(simT/Ts)*100))))
    
end

results_mpc.U       = implementedU;
results_mpc.state   = state;

results_mpc.title = "MPC controller drone";

delete(f)
end

