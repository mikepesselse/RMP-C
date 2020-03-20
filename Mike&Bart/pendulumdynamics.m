function dx =  pendulumdynamics(x, u, g, l, b_a, b_p, m, M)

%% Pendulum Dynamics without force 
% dx = zeros(4,1);
% dx(1) = x(2);
% dx(2) = (3*(g*sin(x(1))-b_a*x(2))*M+sin(x(1))*cos(x(1))*x(2)^2*l*m-3*cos(x(1))*b_p*x(4)) / (3*cos(x(1))^2*l*m+4*M*l);
% % dx(2) = ( 3*sin(x(1))*cos(x(1))*x(2)^2*l*m+3*M*sin(x(1))*g-3*cos(x(1))*b_a*x(3) ) / ( l*(3*cos(x(1))^2*m+4*M) );
% dx(3) = x(4);
% dx(4) = u;

%% Pendulum Dynamics with force 
dx = zeros(4,1);
dx(1) = x(2);
dx(2) = ((3*m*l*x(2)^2*sin(x(1))-3*b_p*x(4)+3*u)*cos(x(1))-3*(M+m)*(b_a*x(2)-g*sin(x(1)))) / (l*(3*cos(x(1))^2*m+4*(M+m))) ;
dx(3) = x(4);
dx(4) = (3*(b_a*x(2)-g*sin(x(1)))*m*cos(x(1))+4*m*l*x(2)^2*sin(x(1))-4*b_p*x(2)+4*u) / (3*(cos(x(1))^2)*m+4*(M+m));

end