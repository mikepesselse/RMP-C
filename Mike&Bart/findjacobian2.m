function [jac] = findjacobian(linstate)
el = 0.046;
Ix = 1.43e-5;
Iy = 1.43e-5;
Iz = 2.89e-5;
Jr = 1e-10;
m = 0.03;
g = 9.81;
a1 = (Iy-Iz)/Ix;
a2 = Jr/Ix;
a3 = (Iz-Ix)/Iy;
a4 = Jr/Iy;
a5 = (Ix-Iy)/Iz;
omr = 1e2; %???
b1 = el/Ix;
b2 = el/Iy;
b3 = 1/Iz;

syms phi dphi theta dtheta psii dpsii z dz x dx y dy u1 u2 u3 u4

dx1 = dphi;
dx2 = dtheta*dpsii*a1 + dtheta*a2*omr + b1*u2;
dx3 = dtheta;
dx4 = dphi*dpsii*a3 - dphi*a4*omr + b2*u3;
dx5 = dpsii;
dx6 = dtheta*dphi*a5 + b3*u4;
dx7 = dz;
dx8 = g - (cos(phi)*cos(theta))/m*u1;
dx9 = dx;
dx10 = (cos(phi)*sin(theta)*cos(psii)+sin(phi)*sin(psii))/m*u1;
dx11 = dy;
dx12 = (cos(phi)*sin(theta)*cos(psii)-sin(phi)*sin(psii))/m*u1;

jac = jacobian([dx1 dx2 dx3 dx4 dx5 dx6 dx7 dx8 dx9 dx10 dx11 dx12], [phi dphi theta dtheta psii dpsii z dz x dx y dy u1 u2 u3 u4]);
jac = subs(jac, [phi dphi theta dtheta psii dpsii z dz x dx y dy u1 u2 u3 u4], linstate);
end