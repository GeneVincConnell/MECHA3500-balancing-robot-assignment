function [dx, dtheta] = robotFlowPlant_AIgen(x, omega, p)
%ROBOTFLOWPLANT_AIGEN  Nonlinear flow-source (Hamiltonian) model of the
%balancing robot, for simulation (Simulink MATLAB Function block or ode45).
%
%   [dx, dtheta] = robotFlowPlant_AIgen(x, omega, p)
%     x      = [phi; theta; ptheta]   (same state as classBalancingRobotFlowLinear_AIgen)
%     omega  = dphi, motor shaft speed relative to the chassis [rad/s] (the steppers)
%     p      = [mc; Jc; mw; Jw; l; r; g; b_theta]   (robotLQG_setup_AIgen builds it)
%     dx     = dx/dt
%     dtheta = d(theta)/dt, what the gyro measures
%
%   Flat ground (alpha = 0). With the wheel angle phi + theta and s = r*(phi + theta):
%     a = Jw + (mw + mc)*r^2,  b = Jc + mc*l^2,  c = mc*l*r*cos(theta)
%     M = [a, a + c; a + c, a + b + 2c]
%     dtheta  = (ptheta - M21*omega)/M22
%     dptheta = dL/dtheta - b_theta*dtheta
%             = -mc*l*r*sin(theta)*(omega + dtheta)*dtheta + mc*g*l*sin(theta) - b_theta*dtheta
%   Linearised about x = 0 this gives exactly the A and B of
%   classBalancingRobotFlowLinear_AIgen (checked numerically, 14 Nov 2026).
%
% AI-GENERATED (Claude, 2026-11-14). Written by Claude (Anthropic) for Gene
% Connell's MCHA3500 balancing robot from the flow-source model of MCHA3500
% Assignment 1, Problem 2. Not copied from any outside source. It replaces the
% EV3 Gyroboy's plant equations (a DC-motor torque model) in the Simulink
% model built by build_robotLQG_sim_AIgen.m.
%#codegen

mc  = p(1);  Jc = p(2);  mw = p(3);  Jw  = p(4);
l   = p(5);  r  = p(6);  g  = p(7);  bth = p(8);

th  = x(2);
pth = x(3);

a  = Jw + (mw + mc)*r^2;
b  = Jc + mc*l^2;
c  = mc*l*r*cos(th);
dc = -mc*l*r*sin(th);                 % dc/dtheta

M21 = a + c;
M22 = a + b + 2*c;

dtheta = (pth - M21*omega)/M22;
dpth   = dc*(omega + dtheta)*dtheta + mc*g*l*sin(th) - bth*dtheta;

dx = [omega; dtheta; dpth];
end
