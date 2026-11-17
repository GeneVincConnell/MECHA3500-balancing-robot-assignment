%% run_robotLQG_sim_AIgen.m
% AI-GENERATED (Claude, 2026-11-14; updated 2026-11-15: torque checked
% against the pull-out curve at the actual wheel speed)
% Written by Claude (Anthropic) for Gene Connell's MCHA3500 balancing robot.
%
% Simulates robotLQG_sim.slx (building it first if it is not there) and plots
% the tilt (true, estimated, measured), wheel angle against the reference,
% wheel speed against the stepper limit, and the torque each motor must give
% against what it has at that speed. The torque is the real check on the
% stepper model: above the pull-out torque the real motors would miss steps.
%   tau (both motors) = a*domega + (a + c)*ddtheta + dc*dtheta^2,
%   a = Jw + (mw + mc)*r^2, c = mc*l*r*cos(theta), dc = -mc*l*r*sin(theta)
% (from the Lagrangian of the flow-source model, phi coordinate).

mdl  = 'robotLQG_sim';
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
addpath(here);
if ~exist(fullfile(here, [mdl '.slx']), 'file')
    build_robotLQG_sim_AIgen;
end
robotLQG_setup_AIgen;
out = sim(mdl);

phiRefLog = getLog(out, 'log_phiRef');
wLog      = getLog(out, 'log_omega');
xLog      = getLog(out, 'log_x');
dthLog    = getLog(out, 'log_dtheta');
yLog      = getLog(out, 'log_y');
xhLog     = getLog(out, 'log_xhat');

t     = xLog.time;
X     = xLog.signals.values;              % [phi, theta, ptheta]
Xh    = xhLog.signals.values;
Y     = yLog.signals.values;              % [phi, theta, dtheta] as measured
w     = wLog.signals.values;
dth   = dthLog.signals.values;
phiR  = phiRefLog.signals.values;

% Torque each motor must give
a   = p(4) + (p(3) + p(1))*p(6)^2;
c   = p(1)*p(5)*p(6)*cos(X(:, 2));
dc  = -p(1)*p(5)*p(6)*sin(X(:, 2));
dw  = [0; diff(w)]/Ts;
ddt = gradient(dth, t);
tau = (a*dw + (a + c).*ddt + dc.*dth.^2)/2;
% What each motor has at the speed it is turning (stepperLimits_AIgen)
tauPOw  = interp1(lim.w, lim.tauPO, min(abs(w), lim.w(end)));   % pull-out
tauUseW = tauPOw/motor.SF;                                     % design limit

fell = any(abs(X(:, 2)) > pi/4);
fprintf('\n%s: %s. Max tilt %.2f deg, max wheel speed %.1f rad/s (limit %g),\n', mdl, ...
    ternary(fell, 'FELL OVER', 'balanced'), max(abs(X(:, 2)))*180/pi, max(abs(w)), omegaMax);
fprintf(['  max torque per motor %.3f N m; worst case %.0f %% of the pull-out torque at that speed ' ...
         '(design limit %.0f %%), final wheel angle %.3f rad (ref %.3f)\n'], ...
    max(abs(tau)), 100*max(abs(tau)./tauPOw), 100/motor.SF, X(end, 1), phiR(end));

figure('Name', 'robotLQG_sim');
subplot(4, 1, 1);
plot(t, Y(:, 2)*180/pi, '.', 'Color', [0.75 0.75 0.75], 'MarkerSize', 3); hold on
plot(t, X(:, 2)*180/pi, 'k-', t, Xh(:, 2)*180/pi, 'b--', 'LineWidth', 1.2); hold off; grid on
ylabel('\theta (deg)'); legend('measured', 'true', 'Kalman estimate', 'Location', 'best');
title(sprintf('LQG with integral action: start at %.0f deg, tilt offset %.1f deg', ...
    x0(2)*180/pi, thetaBiasUsed*180/pi));
subplot(4, 1, 2);
plot(t, X(:, 1), 'k-', t, phiR, 'r--', 'LineWidth', 1.2); grid on
ylabel('\phi (rad)'); legend('wheel angle', 'reference', 'Location', 'best');
subplot(4, 1, 3);
plot(t, w, 'k-', 'LineWidth', 1.2); hold on
plot(t([1 end]), omegaMax*[1 1], 'r:', t([1 end]), -omegaMax*[1 1], 'r:'); hold off; grid on
ylabel('\omega (rad/s)'); legend('stepper speed', 'limit', 'Location', 'best');
subplot(4, 1, 4);
h1 = plot(t, tau, 'k-', 'LineWidth', 1.2); hold on
h2 = plot(t, [tauPOw, -tauPOw], 'r-');
h3 = plot(t, [tauUseW, -tauUseW], 'r:'); hold off; grid on
ylabel('torque/motor (N m)'); xlabel('time (s)');
legend([h1, h2(1), h3(1)], {'needed', 'pull-out at this speed', ...
    sprintf('pull-out / SF %.1f', motor.SF)}, 'Location', 'best');

%% Local functions
function v = getLog(out, name)
% To Workspace data, whether sim returned a SimulationOutput or not
if isa(out, 'Simulink.SimulationOutput')
    v = out.get(name);
else
    v = evalin('base', name);
end
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
