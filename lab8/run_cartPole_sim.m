Claude















Run cartpole sim · M
clear
clc
close all
%   Task 1.7 / 2.5 : controlType = 'None',        alpha = 0 
%   Task 2.6       : controlType = 'Regulation',  alpha = 0
%   Task 2.7       : controlType = 'Regulation',  alpha = 5*pi/180
%   Task 2.10      : controlType = 'Integrator',  alpha = 5*pi/180 
controlType = 'Integrator';


alpha = 5*pi/180;               % ground angle
 
mc = 1.9472;                    % cart mass
mp = 35e-3;                     % pendulum mass
l = 0.2864;                     % pendulum length
b_s = 9.0702;                   % cart damping
b_theta = 0.0037;               % pendulum damping
g = 9.8;                        % gravity
 
%% System model: nonlinear cart-pole (Lab 2)

% M(q)*ddq + C(q,dq)*dq + D*dq + grav(q) = [F; 0],   x = [s; theta; ds; dtheta]
M    = @(q) [mc + mp,            mp*l*cos(q(2));
             mp*l*cos(q(2)),     mp*l^2];
C    = @(q,dq) [0, -mp*l*sin(q(2))*dq(2);
                0,  0];
D    = diag([b_s, b_theta]);
grav = @(q,a) [(mc + mp)*g*sin(a);
               -mp*g*l*sin(q(2) - a)];
f    = @(x,u,a) [x(3:4);
                 M(x(1:2)) \ ([u; 0] - C(x(1:2),x(3:4))*x(3:4) - D*x(3:4) - grav(x(1:2),a))];
dx   = @(x,u) f(x,u,alpha);     % plant on this run's ground angle
 


%% Linearised model about the upright position, alpha = 0 (Problem 1)
A = [0  0                       1             0;
     0  0                       0             1;
     0  -mp*g/mc                -b_s/mc       b_theta/(mc*l);
     0  (mc + mp)*g/(mc*l)      b_s/(mc*l)    -(mc + mp)*b_theta/(mc*mp*l^2)];
B = [0; 0; 1/mc; -1/(mc*l)];
 



%% Define regulation output (cart position) and reference
C_r = [1 0 0 0];
D_r = 0;
y_ref = 0;                    
% Compute feed-forward gains
N_ff = [A B;
        C_r D_r] \ [zeros(4,1); 1];
N_x = N_ff(1:4);
N_u = N_ff(5);
% Compute x and u star
xStar = N_x*y_ref;
uStar = N_u*y_ref;




%% Design controller


switch controlType
    
    
    
    
    case 'Regulation'           
        T = 1/10;               % 10 Hz
        Q = diag([10 1 1 1]);
        R = 0.1;
        N = zeros(4,1);
        K = lqrd(A,B,Q,R,N,T);
       
        
        % Define control law
        u  = @(x,z) uStar - K*(x - xStar);
        zp = @(z,x,ub) z;
        Ai = A;  Bi = B;        
        
        
        
 
    case 'Integrator' 
        T = 1/200;              % 200 Hz
        
        
        % Build augmented system matrices
        Ai = [A zeros(4,1);
              C_r 0];
        Bi = [B; D_r];
        Q = diag([10 1 1 1 10]);
        R = 0.01;
        N = zeros(5,1);
        K = lqrd(Ai,Bi,Q,R,N,T);
        
        
        
        % Define control law and integrator update
        u  = @(x,z) uStar - K*[(x - xStar); z];
        zp = @(z,x,ub) z + T*C_r*(x - xStar) + T*D_r*ub;
 
    otherwise                   % Task 1.7 / 2.5: no control input
        T = 1/10;
        u  = @(x,z) 0;
        zp = @(z,x,ub) z;
end
 





%% Simulate system
% The controller updates every T seconds and holds u in between, while the
% nonlinear plant is integrated with ode45 over each interval.
t_sim = [0:T:5];
x = zeros(length(t_sim), 4);    % state at each control update
z = zeros(length(t_sim), 1);    % integrator state
F = zeros(length(t_sim)-1, 1);  % control force over each interval
x(1,:) = [0, 1*pi/180, 0, 0];   % s, theta, ds, dtheta
options = odeset('RelTol',1e-6);
 
t_plot = t_sim(1);              % full-resolution results for plotting
x_plot = x(1,:);
for i = 1:length(t_sim)-1
    xk = x(i,:).';
    % Control action, held constant over [t_sim(i), t_sim(i+1)]
    F(i) = u(xk, z(i));
    % Integrator update for the next sample
    z(i+1) = zp(z(i), xk, F(i) - uStar);
    % Plant over the interval
    ode_wrap = @(t,x) dx(x, F(i));
    [t_i, x_i] = ode45(ode_wrap, [t_sim(i) t_sim(i+1)], xk, options);
    x(i+1,:) = x_i(end,:);
    % Keep every solver point (drop the first: it repeats the last one)
    t_plot = [t_plot; t_i(2:end)];
    x_plot = [x_plot; x_i(2:end,:)];
end
 








%% Plot results
stateNames = {'s', '\theta', 'ds', 'd\theta'};
figure
for i = 1:4
    subplot(4,1,i)
    plot(t_plot, x_plot(:,i), 'LineWidth', 1.5)
    grid on
    xlabel('Time')
    ylabel(stateNames{i})
    legend(stateNames{i})
end
 
% Control action (held between samples)
figure
stairs(t_sim(1:end-1), F, 'LineWidth', 2)
grid on
xlabel('Time')
ylabel('F (N)')
title(['Control action: ' controlType])
 
Claude finished the response
