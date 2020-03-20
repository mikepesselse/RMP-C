function [results_pid] = PID_Controller(Kp, Ki, Kd, simT, Ts, x0, model, ulim, noise)
g = model.g;
l = model.l;
b_a = model.b_a;
b_p = model.b_p;
m = model.m;
M = model.M;

e = zeros(3,1);
Ti = Kp/Ki;
Td = Kd/Kp;
implementedU = [];
x = x0;
U = 0;

f = waitbar(0,'1','Name','Simulating PID controller',...
    'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');
for i = 1:simT/Ts
    if noise
        xn = x + [randn(1)*(0.1/180*pi); randn(1)*(0.1/180*pi); randn(1)*0.001; randn(1)*0.001];
    else
        xn = x;
    end
    
    e(1) = -xn(1);
    U = U + Kp*((1+Ts/Ti+Td/Ts)*e(1)+(-1-2*Td/Ts)*e(2)+Td/Ts*e(3));
    if U < -ulim
        U = -ulim;
        disp('Warning: PID Actuator saturation!')
    elseif U > ulim
        U = ulim;
        disp('Warning: PID Actuator saturation!')
    end
    
    e(3) = e(2);
    e(2) = e(1);
    
    [~,x] = ode45(@(t,xs) pendulumdynamics(xs, U, g, l, b_a, b_p, m, M), [0 Ts], x);
    x = x(end, :)';
    x(1) = wrapToPi(x(1));
    
    implementedU = [implementedU;U(1)];
    state(i, :) = x;
    
    waitbar(i/(simT/Ts),f,sprintf(num2str(ceil(i/(simT/Ts)*100))))
    
    
end

results_pid.U       = implementedU;
results_pid.state   = state;

if noise
    results_pid.title = "PID controller with noise";
else
    results_pid.title = "PID controller without noise";
end

delete(f)
end

