function [CTSS, DTSS] = constructmodel(Ts, linstate)

[jac] = findjacobian(linstate);
CTSS = ss;
CTSS.A = eval(jac(:, 1:13));
CTSS.B = eval(jac(:, 14:17));

% CTSS = ss;
% CTSS.A = [0 1 0 0
%     3*model.g*(model.m+model.M)/(model.l*(7*model.m+4*model.M)) -3*model.b_a*(model.m+model.M)/(model.l*(7*model.m+4*model.M)) 0 -3*model.b_p / (model.l*(7*model.m+4*model.M))
%     0 0 0 1
%     -3*model.g*model.m / (7*model.m+4*model.M) 3*model.b_a*model.m / (7*model.m+4*model.M) 0 -4*model.b_p / (7*model.m+4*model.M)];
% 
% CTSS.B = [0
%     3/(model.l*(7*model.m+4*model.M))
%     0
%     4/(7*model.m+4*model.M)];

CTSS.C = eye(13);

CTSS.D = 0;

% CTSS.Name = 'Inverted Pendulum';
% CTSS.InputName = 'Force';
% CTSS.InputUnit = 'm/s^2';
% CTSS.OutputName = {'Angle', 'Position'};
% CTSS.OutputUnit = {'rad', 'm'};
% CTSS.StateName = {'Angle_p', 'Ang Vel_p', 'Position_c', 'Velocity_c'};
% CTSS.StateUnit = {'rad', 'rad/s', 'm', 'm/s'};


%% Discretize CT SS-model
DTSS = c2d(CTSS, Ts, 'zoh');
% DTSS.Name = 'Inverted Pendulum';
% DTSS.TimeUnit = 'seconds';


end