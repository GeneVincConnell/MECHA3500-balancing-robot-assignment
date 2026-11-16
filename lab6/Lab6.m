% Load motor data
load('motor1VoltageData_1hz')
% Unpack data
time = motorData.time;
voltage = motorData.voltage;
current = motorData.current;
velocity = motorData.velocity;


%linear least squares solution
%https://www.mathworks.com/help/matlab/ref/lsqr.html
%https://www.mathworks.com/help/curvefit/least-squares-fitting.html
X = [current, velocity];      % N-by-2 design matrix
y = voltage;                  % N-by-1 response

%backslash (QR)
b = X \ y;
%recomended by matlab


%lsqr
[b_lsqr, flag, relres, iter] = lsqr(X, y);

Ra = b(1);
Kw = b(2);

V_fit   = Ra*current + Kw*velocity;               % recovered voltage, 1 Hz data
rms_fit = sqrt(mean((voltage - V_fit).^2));       % RMS error of the 1 Hz fit

figure(1)
hold on
grid on
% Plot the input voltage
plot(time,voltage)
% Plot the recovered voltage
plot(time,Ra*current + Kw*velocity,'+')

%validate based on motor data

load('motor1VoltageData_2hz')
time2     = motorData.time(:);
voltage2  = motorData.voltage(:);
current2  = motorData.current(:);
velocity2 = motorData.velocity(:);
V_val   = Ra*current2 + Kw*velocity2;
rms_val = sqrt(mean((voltage2 - V_val).^2));
pct_val = 100*(1 - norm(voltage2 - V_val)/norm(voltage2 - mean(voltage2)));

figure(2)
subplot(2,1,1)
hold on
grid on
plot(time2, voltage2)
plot(time2, V_val, '+')
ylabel('Voltage (V)')
legend('Measured V_{in}', 'R_a I_a + K_\omega \omega_a (1 Hz parameters)')
title('Task 1.4: validation on 2 Hz data')
subplot(2,1,2)
plot(time2, voltage2 - V_val)
grid on
xlabel('Time (s)')
ylabel('Error (V)')






c1 = [0 0.4470 0.7410];         % blue: 1 Hz
c2 = [0.8500 0.3250 0.0980];    % orange: 2 Hz
figure(3)
set(gcf, 'Position', [100 100 900 650])
subplot(2,1,1)
hold on
grid on
plot(time,  voltage,  '-', 'Color', c1, 'LineWidth', 1.2)
plot(time,  V_fit,    '+', 'Color', c1, 'MarkerSize', 4)
plot(time2, voltage2, '-', 'Color', c2, 'LineWidth', 1.2)
plot(time2, V_val,    '+', 'Color', c2, 'MarkerSize', 4)
ylabel('Voltage (V)')
title('Measured vs recovered voltage (R_a, K_\omega from the 1 Hz fit)')
legend('1 Hz measured', '1 Hz recovered (fit)', ...
       '2 Hz measured', '2 Hz recovered (validation)', ...
       'Location', 'southoutside', 'Orientation', 'horizontal')
subplot(2,1,2)
hold on
grid on
plot(time,  voltage  - V_fit, 'Color', c1)
plot(time2, voltage2 - V_val, 'Color', c2)
xlabel('Time (s)')
ylabel('Error (V)')
title('Measured minus recovered voltage')
legend(sprintf('1 Hz, RMS %.2f V', rms_fit), sprintf('2 Hz, RMS %.2f V', rms_val), ...
       'Location', 'southoutside', 'Orientation', 'horizontal')
%% cart system identification

% Load cart response data
load('cartForceData_1_0hz_1_8N')
% Unpack data
time = cartData.time;
force = cartData.force;
velocity = cartData.velocity;

% Interpolate force data-points
F_in = @(t) interp1(time, force, t, 'linear', 'extrap');



tq = linspace(time(1), time(end), 2000);
figure
plot(time, force, 'o', tq, F_in(tq), '-')
xlabel('Time (s)')
ylabel('Force (N)')
legend('Samples', 'F\_in(t)')




%% Functions
function y = cartSim(t_sim,params,F_in)
% Unpack parameters
m = params(1);
b_s = params(2);
% TODO: Define cart ODE. F_in is a function handle
dp = @(t,p) F_in(t) - b_s/m*p;
% Define initial conditions
p0 = 0;
% Run simulation
[~,x] = ode45(dp,t_sim,p0);
% TODO: Return simulated velocity
y = x/m; %(V = momentum/mass)
end