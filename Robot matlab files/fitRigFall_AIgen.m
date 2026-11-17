%% fitRigFall_AIgen.m
% AI-GENERATED (Claude, 2026-11-14)
% Written by Claude (Anthropic) for Gene Connell's MCHA3500 balancing robot.
% Not copied from any outside source; it uses only MATLAB built-ins
% (backslash for the least-squares solve, cumtrapz, ode45, expm).
%
% Fits the robot's fall on the test rig (no wheels, motor shafts held in
% the rig about 10.5 cm off the ground, chassis pivoting about the axle)
% with linear least squares, and compares it with the no-wheel Hamiltonian
% model (classBalancingRobotRigLinear_AIgen) built from the current
% parameter estimates. No friction anywhere.
%
% Input: one or more CSVs saved by logRobotSensors.m (logAll or Fall=true,
% old or new firmware). Several runs are fitted together. Keep these in the
% same folder, or on the MATLAB path:
%   classBalancingRobotRigLinear_AIgen.m, classBalancingRobotFlowLinear_AIgen.m
%
% ---------------------------------------------------------------------------
% Model fitted: a frictionless inverted pendulum about the axle,
%   theta_ddot = lambda^2*sin(theta - theta_eq)
% theta_eq is the tilt reading at which the CoM is straight above the axle
% (it also absorbs any offset in the IMU's zero). Expanding the sine makes
% this linear in two unknowns:
%   theta_ddot = K1*sin(theta) + K2*cos(theta),
%   K1 = lambda^2*cos(theta_eq),  K2 = -lambda^2*sin(theta_eq)
% and the small-angle (linear) version is
%   theta_ddot = lambda^2*(theta - theta_eq).
% Differentiating the gyro would amplify its noise, so both equations are
% integrated once from the start of each run's fit window instead, and the
% gyro rate itself is fitted:
%   dtheta(t) = dtheta0 + K1*int(sin(theta)) + K2*int(cos(theta))
%   dtheta(t) = dtheta0 + lambda^2*int(theta - theta_eq)
% theta is the integrated gyro. The unknowns enter linearly, so each fit is
% one ordinary linear least-squares solve, X\y. With several runs, lambda
% and theta_eq are shared and each run gets its own dtheta0.
%
% Why several runs: in one fall, a slightly different theta_eq with a
% slightly different lambda fits almost as well, so the two trade off.
% Runs that start from different angles pin theta_eq down (or measure it:
% see thetaEqDeg).
%
% Fit window: the end is just before the impact (the fastest sample), or
% before the gyro clips. A falling pendulum only ever speeds up, so the
% start is after the last slow-down (a hand on the robot), and then moved
% later until the fit stops improving: a finger still trailing on the
% robot shows up as a worse fit. Set tWindow to choose it yourself.
%
% What the fall can and cannot tell you: it measures
%   lambda^2 = g/L_eq,   L_eq = J_axle/(mc*l)
% J_axle (inertia about the axle) and mc*l both grow when mass moves up, so
% the fall rate hardly changes with the CoM height l (the bottom-right plot
% shows this). The fall pins down L_eq. Get l from a static balance test,
% put it in lMeasured below, and the fall then gives the inertia Jc.
% ---------------------------------------------------------------------------

%% Settings
csvFiles   = {};    % {} = pick one or more files in a dialog (Ctrl-click for several)
tWindow    = [];    % [] = automatic; or one [t0 t1] row (s) per file
linMaxDeg  = 25;    % the linear (small-angle) fit stops this far from theta_eq
endMarginS = 0.012; % stop this long before the impact or the gyro clipping [s]
rmsTol     = 1.3;   % automatic start: the earliest start whose fit is within this
                    % factor of the best one with a later start
thetaEqDeg = NaN;   % NaN = fit the balance angle. Or the reading at which the robot
                    % balances on the rig [deg] (balance it by hand, read theta in
                    % Live Expressions): one unknown fewer, steadier lambda.
lMeasured  = NaN;   % CoM height above the axle from a static balance test [m];
                    % NaN = use the estimate in classBalancingRobotFlowLinear_AIgen

g   = 9.81;
d2r = 180/pi;

%% Load the runs and choose each fit window
if isempty(csvFiles)
    [fName, fPath] = uigetfile('*.csv', 'Pick the rig fall log(s)', 'MultiSelect', 'on');
    if isequal(fName, 0)
        return
    end
    csvFiles = fullfile(fPath, cellstr(fName));
end
if ischar(csvFiles)
    csvFiles = {csvFiles};
end
nRun = numel(csvFiles);
for k = 1:nRun
    if isempty(tWindow)
        runs(k) = cutRun(csvFiles{k}, [], endMarginS, rmsTol);            %#ok<SAGROW>
    else
        runs(k) = cutRun(csvFiles{k}, tWindow(k, :), endMarginS, rmsTol); %#ok<SAGROW>
    end
end

%% Linear least-squares fits
thEqFix = thetaEqDeg/d2r;                                % NaN = fit it
fitS = llsJoint(runs, 'sine', thEqFix);                  % all runs together

% Each run on its own, for comparison
lamEach = zeros(nRun, 1);
eqEach  = zeros(nRun, 1);
for k = 1:nRun
    f = llsJoint(runs(k), 'sine', thEqFix);
    lamEach(k) = f.lambda;
    eqEach(k)  = f.thetaEq;
end

% Small-angle linear fit, about theta_eq from the sine fit (fitting theta_eq
% again in this short stretch would trade off against lambda)
iLin = zeros(nRun, 1);
for k = 1:nRun
    a = runs(k).i0;
    near = abs(runs(k).theta(a:runs(k).i1) - fitS.thetaEq) <= linMaxDeg/d2r;
    nNear = find(~near, 1) - 1;
    if isempty(nNear)
        nNear = numel(near);
    end
    iLin(k) = a + nNear - 1;
end
useLin = iLin - [runs.i0]' + 1 >= 10;
w0Lin  = nan(nRun, 1);
if any(useLin)
    fitL = llsJoint(runs(useLin), 'linear', fitS.thetaEq, iLin(useLin));
    w0Lin(useLin) = fitL.w0;
else
    fitL = [];
end

%% Hamiltonian model (no wheels) from the current estimates
rig  = classBalancingRobotRigLinear_AIgen();
lamM = rig.unstablePole();

% Where the CoM could be: move the uncertain 0.65 kg of frame, PCB and
% wiring up and down and see what the model's fall rate does
yMid     = linspace(0, 0.14, 57);
lSweep   = zeros(size(yMid));
lamSweep = zeros(size(yMid));
for k = 1:numel(yMid)
    rk = classBalancingRobotRigLinear_AIgen( ...
            classBalancingRobotFlowLinear_AIgen.estimatedParameters(yMid(k)));
    lSweep(k)   = rk.l;
    lamSweep(k) = rk.unstablePole();
end

% Inertia from the fit, for a given CoM height
if isnan(lMeasured)
    lUse = rig.l;   lNote = 'the estimate';
else
    lUse = lMeasured;   lNote = 'measured';
end
JaxleFit = rig.mc*g*lUse/fitS.lambda2;                   % lambda^2 = mc*g*l/J_axle
JcFit    = JaxleFit - rig.mc*lUse^2 - rig.Jr;

% What that does to the wheeled model the controller will use
robot       = classBalancingRobotFlowLinear_AIgen();
robotFit    = robot;
robotFit.l  = lUse;
robotFit.Jc = JcFit;

%% Report
fprintf('\nRun                                  fit window (s)  from rest (deg)  alone: lambda   theta_eq\n');
for k = 1:nRun
    r = runs(k);
    [~, name] = fileparts(r.file);
    fprintf('%-36s %6.3f - %6.3f  %5.1f to %5.1f    %6.3f /s   %6.2f deg\n', name, ...
        r.t(r.i0), r.t(r.i1), r.dev(r.i0)*d2r, r.dev(r.i1)*d2r, lamEach(k), eqEach(k)*d2r);
end
fprintf('\n                                    lambda    L_eq = g/lambda^2   theta_eq    R^2\n');
fprintf('LLS fit, sine model (all runs)     %6.3f /s   %6.1f cm          %6.2f deg  %.5f\n', ...
    fitS.lambda, 100*g/fitS.lambda2, fitS.thetaEq*d2r, fitS.R2);
if ~isempty(fitL)
    fprintf('LLS fit, linear model (to %2g deg)  %6.3f /s   %6.1f cm          (same)      %.5f\n', ...
        linMaxDeg, fitL.lambda, 100*g/fitL.lambda2, fitL.R2);
else
    fprintf('LLS fit, linear model: no run has 10 samples within %g deg of theta_eq\n', linMaxDeg);
end
fprintf('Hamiltonian model, no wheels       %6.3f /s   %6.1f cm\n', lamM, 100*rig.equivalentLength());
fprintf('  -> the sine fit is %+.1f %% from the model''s fall rate\n', 100*(fitS.lambda/lamM - 1));
if isnan(thetaEqDeg) && nRun == 1
    fprintf(['  (one run: theta_eq and lambda trade off. Fit two or more runs together, or set\n' ...
             '   thetaEqDeg to the measured balance reading.)\n']);
end

near = abs(lamSweep - fitS.lambda) <= 0.02*fitS.lambda;
[lamMax, iMax] = max(lamSweep);
if any(near)
    fprintf('\nCoM height: l from %.1f to %.1f cm gives a model fall rate within 2 %% of the fit,\n', ...
        100*min(lSweep(near)), 100*max(lSweep(near)));
    fprintf('so the fall alone cannot pin down l (the model''s fastest is %.2f /s, at l = %.1f cm).\n', ...
        lamMax, 100*lSweep(iMax));
else
    fprintf(['\nCoM height: no l in the sweep (%.1f to %.1f cm) gives a model fall rate within 2 %% ' ...
             'of the fit\n(the model''s fastest is %.2f /s, at l = %.1f cm): the inertia or mass ' ...
             'estimate is off.\n'], 100*min(lSweep), 100*max(lSweep), lamMax, 100*lSweep(iMax));
end
fprintf('\nWith l = %.2f cm (%s), the fit gives\n', 100*lUse, lNote);
fprintf('  J_axle = mc*g*l/lambda^2      = %.5f kg m^2 (model %.5f)\n', JaxleFit, rig.Jaxle());
fprintf('  Jc     = J_axle - mc*l^2 - Jr = %.5f kg m^2 (model %.5f)\n', JcFit, rig.Jc);
fprintf('  wheeled robot''s unstable pole: %.3f /s (model %.3f /s)\n\n', ...
    robotFit.unstablePole(), robot.unstablePole());

%% Plots
col  = lines(nRun);
figure('Name', 'Rig fall: least-squares fit vs Hamiltonian model');

% theta against time from each window start
subplot(2, 2, [1 2]); hold on
odeOpt = odeset('RelTol', 1e-9, 'AbsTol', 1e-11);
runNames = cell(1, nRun);
for k = 1:nRun
    r   = runs(k);
    idx = (r.i0:r.i1)';
    tau = r.t(idx) - r.t(r.i0);
    sel = r.t >= r.t(r.i0) - 0.15 & r.t <= r.t(r.i1) + 0.05;
    [~, xS] = ode45(@(tt, x) [x(2); fitS.lambda2*sin(x(1) - fitS.thetaEq)], ...
                    tau, [r.theta(r.i0); fitS.w0(k)], odeOpt);
    [thM, ~] = rig.freeResponse(tau, r.theta(r.i0) - fitS.thetaEq, r.w(r.i0));
    hData = plot(r.t(sel) - r.t(r.i0), r.theta(sel)*d2r, '.', 'Color', 0.4*col(k, :) + 0.6, ...
                 'MarkerSize', 6);                       % light, so the fits show
    hSine = plot(tau, xS(:, 1)*d2r, '-', 'Color', col(k, :), 'LineWidth', 1.5);
    hLin  = [];
    if useLin(k)
        thL = fitS.thetaEq + (r.theta(r.i0) - fitS.thetaEq)*cosh(fitL.lambda*tau) ...
              + w0Lin(k)/fitL.lambda*sinh(fitL.lambda*tau);
        hLin = plot(tau, thL*d2r, '--', 'Color', col(k, :), 'LineWidth', 1.2);
    end
    hModel = plot(tau, (thM + fitS.thetaEq)*d2r, '-.', 'Color', 0.5*col(k, :), 'LineWidth', 1.5);
    if k == 1
        hLeg = [hData, hSine, hLin, hModel];
    end
    [~, runNames{k}] = fileparts(r.file);
end
hold off; grid on
xlabel('time from the start of the fit window (s)'); ylabel('\theta (deg), integrated gyro');
linLabel = {};
if useLin(1)
    linLabel = {sprintf('LLS fit, linear model (to %g deg): \\lambda = %.2f /s', linMaxDeg, fitL.lambda)};
end
labels = [{'measured', sprintf('LLS fit, sine model: \\lambda = %.2f /s', fitS.lambda)}, ...
          linLabel, {sprintf('Hamiltonian model (estimate): \\lambda = %.2f /s', lamM)}];
legend(hLeg, labels, 'Location', 'best');
title(['Fall on the rig. Runs: ', strjoin(runNames, ', ')], 'Interpreter', 'none');

% theta_ddot against the angle from balance: the slope is lambda^2 = mc*g*l/J_axle
subplot(2, 2, 3); hold on
xAll = [];
for k = 1:nRun
    r    = runs(k);
    idx  = (r.i0:r.i1)';
    thdd = gradient(movmean(r.w, r.nAvg), r.t);           % ~10 ms smoothing, plot only
    x    = r.theta(idx) - fitS.thetaEq;
    plot(x*d2r, thdd(idx), '.', 'Color', col(k, :), 'MarkerSize', 7);
    xAll = [xAll; x];                                    %#ok<AGROW>
end
xs = linspace(min([0; xAll]), max([0; xAll]), 200)';
hC = plot(xs*d2r, fitS.lambda2*sin(xs), 'k-', 'LineWidth', 1.5);
hM = plot(xs*d2r, lamM^2*xs, 'r-.', 'LineWidth', 1.5);
if ~isempty(fitL)
    hL = plot(xs*d2r, fitL.lambda2*xs, 'k--', 'LineWidth', 1.2);
    legend([hC hL hM], {'sine fit', 'linear fit', 'model'}, 'Location', 'best');
else
    legend([hC hM], {'sine fit', 'model'}, 'Location', 'best');
end
hold off; grid on
xlabel('\theta - \theta_{eq} (deg)'); ylabel('d^2\theta/dt^2 (rad/s^2)');
title('Slope near 0 = \lambda^2 = m_c g l / J_{axle}');

% Model fall rate against CoM height
subplot(2, 2, 4); hold on
plot(100*lSweep, lamSweep, 'r-', 'LineWidth', 1.5);
xl = 100*[min(lSweep), max(lSweep)];
plot(xl, fitS.lambda*[1 1], 'k-');
plot(100*rig.l, lamM, 'ro', 'MarkerFaceColor', 'r');
legText = {'model, moving the unknown 0.65 kg', 'sine fit', 'current estimate'};
if ~isnan(lMeasured)
    yl = ylim;
    plot(100*lMeasured*[1 1], yl, 'm-', 'LineWidth', 1.5);
    legText{end + 1} = 'l measured';
end
hold off; grid on
xlabel('CoM height above the axle, l (cm)'); ylabel('\lambda (1/s)');
legend(legText, 'Location', 'best');
title('The fall rate barely depends on l');

%% Local functions
function run = cutRun(file, tWin, endMarginS, rmsTol)
% Load one log and choose its fit window (see the header).
T    = readtable(file);
cols = T.Properties.VariableNames;
need = {'time_s', 'theta_rad', 'theta_dot_rad_s', 'gyro_x_raw'};
if ~all(ismember(need, cols))
    error('fitRigFall:columns', '%s needs the columns %s.', file, strjoin(need, ', '));
end
t       = T.time_s(:);
w       = T.theta_dot_rad_s(:);      % gyro rate, bias removed, + = forwards [rad/s]
thFw    = T.theta_rad(:);            % firmware tilt (complementary filter) [rad]
clipped = abs(T.gyro_x_raw(:)) >= 32767;
thInt   = cumtrapz(t, w);            % integrated gyro; its offset is set below
nAvg    = max(1, round(0.010/median(diff(t))));   % ~10 ms of samples

if isempty(tWin)
    % End: the fastest unclipped sample is just before the impact; if the
    % gyro clipped first (old firmware, +/-250 deg/s), stop before that
    wAbs = abs(w);
    wAbs(clipped) = 0;
    [~, iPk] = max(wAbs);
    iClip = find(clipped, 1);
    if ~isempty(iClip) && iClip <= iPk + 1
        iEnd = iClip - 1;
    else
        iEnd = iPk;
    end
    i1 = find(t <= t(iEnd) - endMarginS, 1, 'last');

    % The angle is anchored to the firmware's (accelerometer-corrected)
    % reading at the last moment the robot was held still
    iQuiet = find(abs(w(1:iEnd)) < 0.02, 1, 'last');
    if isempty(iQuiet)
        error('fitRigFall:window', ['%s: could not find where the robot was held still before ' ...
            'the fall. Set tWindow by hand.'], file);
    end
    theta   = thInt - thInt(iQuiet) + thFw(iQuiet);
    fallDir = sign(w(iEnd));
    dev     = fallDir*(theta - theta(iQuiet));           % distance fallen from rest

    % Start: after the last slow-down, then later until the fit stops improving
    acc  = fallDir*gradient(movmean(w, nAvg), t);
    iDec = find(acc(1:i1) <= 0, 1, 'last');
    if isempty(iDec)
        iDec = iQuiet;
    end
    jLo = iDec + nAvg;
    jHi = min(find(dev(1:i1) <= 30/180*pi, 1, 'last'), i1 - 30);
    if isempty(jHi) || jHi < jLo
        jHi = jLo;
    end
    if jLo > i1 - 10
        error('fitRigFall:window', '%s: the free fall is too short to fit. Set tWindow by hand.', file);
    end
    js   = jLo:jHi;
    rmsJ = zeros(size(js));
    for m = 1:numel(js)
        j = js(m);
        rmsJ(m) = sineRms(t(j:i1), theta(j:i1), w(j:i1));
    end
    i0 = js(find(rmsJ <= rmsTol*min(rmsJ), 1));
else
    i0 = find(t >= tWin(1), 1);
    i1 = find(t <= tWin(2), 1, 'last');
    iQuiet  = i0;
    theta   = thInt - thInt(i0) + thFw(i0);
    fallDir = sign(w(i1));
    dev     = fallDir*(theta - theta(i0));
end
run = struct('file', file, 't', t, 'theta', theta, 'w', w, 'dev', dev, ...
             'i0', i0, 'i1', i1, 'iQuiet', iQuiet, 'nAvg', nAvg);
end

function r = sineRms(t, th, w)
% RMS residual of the sine fit of one stretch (used to choose the start)
X = [ones(size(t)), cumtrapz(t, sin(th)), cumtrapz(t, cos(th))];
r = sqrt(mean((w - X*(X\w)).^2));
end

function f = llsJoint(runs, kind, thetaEq, lastIdx)
% Linear least squares over one or more runs (see the header): lambda and
% theta_eq shared, one starting rate dtheta0 per run. thetaEq = NaN fits
% the balance angle too. lastIdx (optional) ends each run's stretch early.
nRun  = numel(runs);
fitEq = isnan(thetaEq);
X = [];  y = [];
for k = 1:nRun
    a = runs(k).i0;
    b = runs(k).i1;
    if nargin > 3
        b = lastIdx(k);
    end
    t  = runs(k).t(a:b);
    th = runs(k).theta(a:b);
    oneHot = zeros(numel(t), nRun);
    oneHot(:, k) = 1;
    if strcmp(kind, 'sine')
        if fitEq
            R = [cumtrapz(t, sin(th)), cumtrapz(t, cos(th))];
        else
            R = cumtrapz(t, sin(th - thetaEq));
        end
    else
        if fitEq
            R = [cumtrapz(t, th), -(t - t(1))];
        else
            R = cumtrapz(t, th - thetaEq);
        end
    end
    X = [X; oneHot, R];                                  %#ok<AGROW>
    y = [y; runs(k).w(a:b)];                             %#ok<AGROW>
end
c = X\y;                                                 % ordinary linear least squares
p = c(nRun + 1:end);
if fitEq && strcmp(kind, 'sine')
    f.lambda2 = hypot(p(1), p(2));
    f.thetaEq = atan2(-p(2), p(1));
elseif fitEq
    f.lambda2 = p(1);
    f.thetaEq = p(2)/p(1);
else
    f.lambda2 = p(1);
    f.thetaEq = thetaEq;
end
if f.lambda2 <= 0
    warning('fitRigFall:lambda', 'The %s fit found no unstable fall in this window.', kind);
end
f.lambda = sqrt(max(f.lambda2, 0));
f.w0     = c(1:nRun);                                    % fitted rate at each window start
res      = y - X*c;
f.R2     = 1 - sum(res.^2)/sum((y - mean(y)).^2);
f.rms    = sqrt(mean(res.^2));
end
