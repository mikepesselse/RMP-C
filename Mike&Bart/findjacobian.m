function [jac] = findjacobian(linstate)
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

% QUADEOM_READONLY Solve quadrotor equation of motion
%   quadEOM_readonly calculate the derivative of the state vector
%
% INPUTS:
% t      - 1 x 1, time
% s      - 13 x 1, state vector = [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
% F      - 1 x 1, thrust output from controller (only used in simulation)
% M      - 3 x 1, moments output from controller (only used in simulation)
% params - struct, output from crazyflie() and whatever parameters you want to pass in
%
% OUTPUTS:
% sdot   - 13 x 1, derivative of state vector s
%
% NOTE: You should not modify this function
% See Also: quadEOM_readonly, crazyflie

%************ EQUATIONS OF MOTION ************************
% Limit the force and moments due to actuator limits
A = [0.25,                      0, -0.5/params.arm_length;
     0.25,  0.5/params.arm_length,                      0;
     0.25,                      0,  0.5/params.arm_length;
     0.25, -0.5/params.arm_length,                      0];

% prop_thrusts = A*[F;M(1:2)]; % Not using moment about Z-axis for limits
% prop_thrusts_clamped = max(min(prop_thrusts, params.maxF/4), params.minF/4);

B = [                 1,                 1,                 1,                  1;
                      0, params.arm_length,                 0, -params.arm_length;
     -params.arm_length,                 0, params.arm_length,                 0];
% F = B(1,:)*prop_thrusts_clamped;
% M = [B(2:3,:)*prop_thrusts_clamped; M(3)];

syms s1 s2 s3 s4 s5 s6 s7 s8 s9 s10 s11 s12 s13 F M1 M2 M3
% Assign states
x = s1;
y = s2;
z = s3;
xdot = s4;
ydot = s5;
zdot = s6;
qW = s7;
qX = s8;
qY = s9;
qZ = s10;
p = s11;
q = s12;
r = s13;

quat = [qW; qX; qY; qZ];
bRw = QuatToRot(quat);
wRb = bRw';

% Acceleration
accel = 1 / params.mass * (wRb * [0; 0; F] - [0; 0; params.mass * params.grav]);

% Angular velocity
K_quat = 2; %this enforces the magnitude 1 constraint for the quaternion
quaterror = 1 - (qW^2 + qX^2 + qY^2 + qZ^2);
qdot = -1/2*[0, -p, -q, -r;...
             p,  0, -r,  q;...
             q,  r,  0, -p;...
             r, -q,  p,  0] * quat + K_quat*quaterror * quat;

% Angular acceleration
omega = [p;q;r];
pqrdot   = params.invI * ([M1; M2; M3] - cross(omega, params.I*omega));

% Assemble sdot
% sdot = zeros(13,1);
sdot1  = xdot;
sdot2  = ydot;
sdot3  = zdot;
sdot4  = accel(1);
sdot5  = accel(2);
sdot6  = accel(3);
sdot7  = qdot(1);
sdot8  = qdot(2);
sdot9  = qdot(3);
sdot10 = qdot(4);
sdot11 = pqrdot(1);
sdot12 = pqrdot(2);
sdot13 = pqrdot(3);

jac = jacobian([sdot1 sdot2 sdot3 sdot4 sdot5 sdot6 sdot7 sdot8 sdot9 sdot10 sdot11 sdot12 sdot13], [s1 s2 s3 s4 s5 s6 s7 s8 s9 s10 s11 s12 s13 F M1 M2 M3]);
jac = subs(jac, [s1 s2 s3 s4 s5 s6 s7 s8 s9 s10 s11 s12 s13 F M1 M2 M3], linstate);
end