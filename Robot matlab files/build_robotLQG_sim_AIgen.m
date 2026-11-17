%% build_robotLQG_sim_AIgen.m
% AI-GENERATED (Claude, 2026-11-14)
% Written by Claude (Anthropic) for Gene Connell's MCHA3500 balancing robot.
%
% Builds robotLQG_sim.slx in this folder: an LQG controller with integral
% action (steady-state Kalman filter + LQR) closed around the nonlinear
% flow-source model of the robot, with the steppers' speed and acceleration
% limits and the IMU/encoder quantisation, noise and tilt offset.
% Run it once (or again after changing the layout); the parameters live in
% robotLQG_setup_AIgen.m, which the model runs itself before each simulation.
% Then simulate and plot with run_robotLQG_sim_AIgen.
%
% Layout follows the EV3 Gyroboy LQG model with integral action
% (LQG_IA_NPC/gb_lqg_ia_npc.slx): a Control subsystem (Kalman filter as a
% Discrete State-Space block whose state is the estimate, an integrator,
% the LQR gain), a plant subsystem (plant equations in a MATLAB Function
% block feeding an Integrator), sensor and actuator non-ideality, and To
% Workspace logging.
%   https://github.com/jakubk28/LegoMindstormsEV3/tree/1d4c2ae5f93b4aafa8b2fe3e427c627b9f0127fe/LQG_IA_NPC
%   Copyright (c) 2019, Timothy Hughes. BSD licence.
% No blocks or code are copied from it; this script builds a new model.
% Removed relative to Gyroboy: the LEGO EV3 hardware variants, keyboard /
% gamepad / UDP input, yaw (turning) dynamics and controller, gyro drift
% calibration and the DC-motor plant. Added: the flow-source plant with the
% stepper speed as input, and a controller designed on the stepper
% acceleration with the speed and acceleration limits inside it.
%
% Signals (top level)
%   phi_ref  -> Control -> omega_cmd -> Stepper motors -> omega -> Robot dynamics
%   Robot dynamics -> x = [phi; theta; ptheta], dtheta -> Sensors -> y = [phi; theta; dtheta]
%   y and omega -> Control (Kalman filter)
%
% For MPC: replace the Control subsystem with an MPC Controller block (inputs
% y, phi_ref; output omega_cmd). See the MPC section of robotLQG_setup_AIgen.m.
%
% Needs Simulink and the Control System Toolbox. Tested here only in parts
% (design and closed loop in GNU Octave); the model itself is built and
% checked in MATLAB the first time you run this.

mdl = 'robotLQG_sim';
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
addpath(here);
robotLQG_setup_AIgen;              % workspace variables the blocks use

% Start from scratch; keep a copy of an existing model
if bdIsLoaded(mdl), close_system(mdl, 0); end
mdlFile = fullfile(here, [mdl '.slx']);
if exist(mdlFile, 'file')
    backup = fullfile(here, sprintf('%s_backup_%s.slx', mdl, datestr(now, 'yyyymmdd_HHMMSS')));
    movefile(mdlFile, backup);
    fprintf('Moved the old %s.slx to %s\n', mdl, backup);
end
new_system(mdl);
open_system(mdl);
set_param(mdl, 'PreLoadFcn', 'robotLQG_setup_AIgen;', ...
               'InitFcn',    'robotLQG_setup_AIgen;', ...
               'StopTime',   'Tstop', ...
               'Solver',     'ode45', ...
               'MaxStep',    num2str(Ts), ...
               'RelTol',     '1e-6', ...
               'ReturnWorkspaceOutputs', 'on');

%% Top level
top = mdl;
addBlk(top, 'phi_ref target', 'simulink/Sources/Step', [30 40 60 70], ...
    'Time', 'phiRefStepTime', 'Before', '0', 'After', 'phiRefStep', 'SampleTime', 'Ts');
addBlk(top, 'phi_ref ramp', 'simulink/Discontinuities/Rate Limiter', [100 40 140 70], ...
    'RisingSlewLimit', 'phiRefRate', 'FallingSlewLimit', '-phiRefRate', ...
    'SampleTimeMode', 'inherited', 'InitialCondition', '0');
subsys(top, 'Control',        [200  30 330 130]);
subsys(top, 'Stepper motors', [400  40 500  90]);
subsys(top, 'Robot dynamics', [560  40 680 110]);
subsys(top, 'Sensors',        [560 200 680 260]);
addBlk(top, 'theta in deg', 'simulink/Math Operations/Gain', [760 30 810 60], ...
    'Gain', '[0 180/pi 0]', 'Multiplication', 'Matrix(K*u)');
addBlk(top, 'Tilt (deg) and wheel speed (rad per s)', 'simulink/Sinks/Scope', [860 30 900 90], ...
    'NumInputPorts', '2');
logs = {'log_phiRef', 'log_omega', 'log_x', 'log_dtheta', 'log_y', 'log_xhat', 'log_z'};
for i = 1:numel(logs)
    addBlk(top, logs{i}, 'simulink/Sinks/To Workspace', [960 20+50*i 1040 40+50*i], ...
        'VariableName', logs{i}, 'SaveFormat', 'Structure With Time', 'SampleTime', 'Ts');
end

%% Control: Kalman filter + LQR with integral action, on the wheel acceleration
c = [top '/Control'];
port(c, 'In', 'y', 1);  port(c, 'In', 'phi_ref', 2);  port(c, 'In', 'omega_applied', 3);
port(c, 'Out', 'omega_cmd', 1);  port(c, 'Out', 'xhat', 2);  port(c, 'Out', 'z', 3);
addBlk(c, 'Kalman input', 'simulink/Signal Routing/Mux', [120 20 125 80], 'Inputs', '2');
kf = addBlk(c, 'Kalman filter', 'simulink/Discrete/Discrete State-Space', [170 30 260 70], ...
    'A', 'kest.A', 'B', 'kest.B', 'C', 'eye(3)', 'D', 'zeros(3,4)', 'SampleTime', 'Ts');
try
    set_param(kf, 'InitialCondition', 'xhat0');
catch
    set_param(kf, 'X0', 'xhat0');              % older releases
end
addBlk(c, 'x_ref', 'simulink/Math Operations/Gain', [170 110 220 140], ...
    'Gain', '[1;0;0]', 'Multiplication', 'Matrix(K*u)');
addBlk(c, 'state error', 'simulink/Math Operations/Sum', [300 40 320 120], 'Inputs', '+-');
addBlk(c, 'select phi', 'simulink/Math Operations/Gain', [170 170 220 200], ...
    'Gain', '[1 0 0]', 'Multiplication', 'Matrix(K*u)');
addBlk(c, 'phi error', 'simulink/Math Operations/Sum', [250 170 270 200], 'Inputs', '+-');
addBlk(c, 'Integral of phi error', 'simulink/Discrete/Discrete-Time Integrator', [300 165 350 205], ...
    'IntegratorMethod', 'Integration: Forward Euler', 'gainval', '1.0', ...
    'InitialCondition', '0', 'SampleTime', 'Ts', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'zLimit', 'LowerSaturationLimit', '-zLimit');
addBlk(c, 'LQR state', 'simulink/Signal Routing/Mux', [400 40 405 200], 'Inputs', '3');
addBlk(c, 'LQR gain', 'simulink/Math Operations/Gain', [440 100 500 140], ...
    'Gain', '-K', 'Multiplication', 'Matrix(K*u)');
addBlk(c, 'Acceleration limit', 'simulink/Discontinuities/Saturation', [530 100 570 140], ...
    'UpperLimit', 'alphaMax', 'LowerLimit', '-alphaMax');
addBlk(c, 'Wheel speed', 'simulink/Discrete/Discrete-Time Integrator', [610 100 660 140], ...
    'IntegratorMethod', 'Integration: Forward Euler', 'gainval', '1.0', ...
    'InitialCondition', '0', 'SampleTime', 'Ts', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'omegaMax', 'LowerSaturationLimit', '-omegaMax');
wire(c, 'omega_applied/1', 'Kalman input/1');
wire(c, 'y/1',             'Kalman input/2');
wire(c, 'Kalman input/1',  'Kalman filter/1');
wire(c, 'Kalman filter/1', 'state error/1');
wire(c, 'phi_ref/1',       'x_ref/1');
wire(c, 'x_ref/1',         'state error/2');
wire(c, 'y/1',             'select phi/1');
wire(c, 'select phi/1',    'phi error/1');
wire(c, 'phi_ref/1',       'phi error/2');
wire(c, 'phi error/1',     'Integral of phi error/1');
wire(c, 'state error/1',   'LQR state/1');
wire(c, 'Wheel speed/1',   'LQR state/2');
wire(c, 'Integral of phi error/1', 'LQR state/3');
wire(c, 'LQR state/1',     'LQR gain/1');
wire(c, 'LQR gain/1',      'Acceleration limit/1');
wire(c, 'Acceleration limit/1', 'Wheel speed/1');
wire(c, 'Wheel speed/1',   'omega_cmd/1');
wire(c, 'Kalman filter/1', 'xhat/1');
wire(c, 'Integral of phi error/1', 'z/1');

%% Stepper motors: the physical speed and acceleration limits
s = [top '/Stepper motors'];
port(s, 'In', 'omega_cmd', 1);  port(s, 'Out', 'omega', 1);
addBlk(s, 'Acceleration limit', 'simulink/Discontinuities/Rate Limiter', [120 30 170 70], ...
    'RisingSlewLimit', 'alphaMax', 'FallingSlewLimit', '-alphaMax', ...
    'SampleTimeMode', 'inherited', 'InitialCondition', '0');
addBlk(s, 'Speed limit', 'simulink/Discontinuities/Saturation', [220 30 260 70], ...
    'UpperLimit', 'omegaMax', 'LowerLimit', '-omegaMax');
wire(s, 'omega_cmd/1', 'Acceleration limit/1');
wire(s, 'Acceleration limit/1', 'Speed limit/1');
wire(s, 'Speed limit/1', 'omega/1');

%% Robot dynamics: nonlinear flow-source model (robotFlowPlant_AIgen.m)
r = [top '/Robot dynamics'];
port(r, 'In', 'omega', 1);  port(r, 'Out', 'x', 1);  port(r, 'Out', 'dtheta', 2);
addBlk(r, 'p', 'simulink/Sources/Constant', [60 140 140 170], 'Value', 'p');
fcnBlk = addBlk(r, 'Plant equations', 'simulink/User-Defined Functions/MATLAB Function', [200 40 300 140]);
setFunctionCode(mdl, getfullname(fcnBlk), sprintf([ ...
    'function [dx, dtheta] = plant(x, omega, p)\n' ...
    '%% Flow-source model of the robot: robotFlowPlant_AIgen.m (AI-generated, Claude).\n' ...
    '%% x = [phi; theta; ptheta], omega = stepper speed relative to the chassis.\n' ...
    '[dx, dtheta] = robotFlowPlant_AIgen(x, omega, p);\n']));
addBlk(r, 'Integrator', 'simulink/Continuous/Integrator', [360 40 400 80], 'InitialCondition', 'x0');
wire(r, 'Integrator/1',      'Plant equations/1');
wire(r, 'omega/1',           'Plant equations/2');
wire(r, 'p/1',               'Plant equations/3');
wire(r, 'Plant equations/1', 'Integrator/1');
wire(r, 'Integrator/1',      'x/1');
wire(r, 'Plant equations/2', 'dtheta/1');

%% Sensors: encoders, IMU tilt estimate, gyro (sampled at Ts)
n = [top '/Sensors'];
port(n, 'In', 'x', 1);  port(n, 'In', 'dtheta', 2);  port(n, 'Out', 'y', 1);
addBlk(n, 'phi', 'simulink/Math Operations/Gain', [100 20 150 50], ...
    'Gain', '[1 0 0]', 'Multiplication', 'Matrix(K*u)');
addBlk(n, 'Encoder counts', 'simulink/Discontinuities/Quantizer', [190 20 240 50], ...
    'QuantizationInterval', 'encQ');
addBlk(n, 'theta', 'simulink/Math Operations/Gain', [100 90 150 120], ...
    'Gain', '[0 1 0]', 'Multiplication', 'Matrix(K*u)');
addBlk(n, 'Tilt offset', 'simulink/Sources/Constant', [100 140 150 160], 'Value', 'thetaBiasUsed');
addBlk(n, 'Tilt noise', 'simulink/Sources/Random Number', [100 180 150 210], ...
    'Mean', '0', 'Variance', 'thetaNoiseVar', 'Seed', '1', 'SampleTime', 'Ts');
addBlk(n, 'Tilt estimate', 'simulink/Math Operations/Sum', [200 90 220 210], 'Inputs', '+++');
addBlk(n, 'Gyro noise', 'simulink/Sources/Random Number', [100 280 150 310], ...
    'Mean', '0', 'Variance', 'gyroNoiseVar', 'Seed', '2', 'SampleTime', 'Ts');
addBlk(n, 'Gyro', 'simulink/Math Operations/Sum', [200 240 220 300], 'Inputs', '++');
addBlk(n, 'Gyro LSB', 'simulink/Discontinuities/Quantizer', [260 250 310 280], ...
    'QuantizationInterval', 'gyroQ');
addBlk(n, 'Measurements', 'simulink/Signal Routing/Mux', [360 20 365 300], 'Inputs', '3');
addBlk(n, 'Sample at Ts', 'simulink/Discrete/Zero-Order Hold', [400 140 440 180], 'SampleTime', 'Ts');
wire(n, 'x/1',              'phi/1');
wire(n, 'phi/1',            'Encoder counts/1');
wire(n, 'Encoder counts/1', 'Measurements/1');
wire(n, 'x/1',              'theta/1');
wire(n, 'theta/1',          'Tilt estimate/1');
wire(n, 'Tilt offset/1',    'Tilt estimate/2');
wire(n, 'Tilt noise/1',     'Tilt estimate/3');
wire(n, 'Tilt estimate/1',  'Measurements/2');
wire(n, 'dtheta/1',         'Gyro/1');
wire(n, 'Gyro noise/1',     'Gyro/2');
wire(n, 'Gyro/1',           'Gyro LSB/1');
wire(n, 'Gyro LSB/1',       'Measurements/3');
wire(n, 'Measurements/1',   'Sample at Ts/1');
wire(n, 'Sample at Ts/1',   'y/1');

%% Top-level wiring
wire(top, 'phi_ref target/1',   'phi_ref ramp/1');
wire(top, 'phi_ref ramp/1',     'Control/2');
wire(top, 'Sensors/1',          'Control/1');
wire(top, 'Stepper motors/1',   'Control/3');
wire(top, 'Control/1',          'Stepper motors/1');
wire(top, 'Stepper motors/1',   'Robot dynamics/1');
wire(top, 'Robot dynamics/1',   'Sensors/1');
wire(top, 'Robot dynamics/2',   'Sensors/2');
wire(top, 'Robot dynamics/1',   'theta in deg/1');
wire(top, 'theta in deg/1',     'Tilt (deg) and wheel speed (rad per s)/1');
wire(top, 'Stepper motors/1',   'Tilt (deg) and wheel speed (rad per s)/2');
wire(top, 'phi_ref ramp/1',     'log_phiRef/1');
wire(top, 'Stepper motors/1',   'log_omega/1');
wire(top, 'Robot dynamics/1',   'log_x/1');
wire(top, 'Robot dynamics/2',   'log_dtheta/1');
wire(top, 'Sensors/1',          'log_y/1');
wire(top, 'Control/2',          'log_xhat/1');
wire(top, 'Control/3',          'log_z/1');

%% Tidy, check, save
for sysPath = {top, c, s, r, n}
    try
        Simulink.BlockDiagram.arrangeSystem(sysPath{1});   % R2018b and later
    catch
    end
end
save_system(mdl, mdlFile);
fprintf('Built and saved %s\n', mdlFile);
try
    set_param(mdl, 'SimulationCommand', 'update');        % compile: catches size and wiring errors
    fprintf('Model compiles. Run run_robotLQG_sim_AIgen to simulate and plot.\n');
catch err
    fprintf(2, 'The model was saved but does not compile yet:\n%s\n', err.message);
end

%% Local functions
function h = addBlk(sys, name, lib, pos, varargin)
% Add a library block at sys/name with a position and parameters.
h = add_block(lib, [sys '/' name], 'Position', pos, varargin{:});
end

function subsys(sys, name, pos)
% Empty subsystem.
add_block('built-in/Subsystem', [sys '/' name], 'Position', pos);
end

function port(sys, kind, name, num)
% Inport ('In') or Outport ('Out') number num.
if strcmp(kind, 'In')
    lib = 'simulink/Sources/In1';   x = 20;
else
    lib = 'simulink/Sinks/Out1';    x = 720;
end
add_block(lib, [sys '/' name], 'Port', num2str(num), ...
    'Position', [x 30+60*num x+30 44+60*num]);
end

function wire(sys, from, to)
add_line(sys, from, to, 'autorouting', 'on');
end

function setFunctionCode(mdl, blkPath, code)
% Put code into a MATLAB Function block (its ports follow the signature).
done = false;
try
    cfg = get_param(blkPath, 'MATLABFunctionConfiguration');   % R2019b and later
    cfg.FunctionScript = code;
    done = true;
catch
end
if ~done
    rt    = sfroot;
    model = rt.find('-isa', 'Simulink.BlockDiagram', 'Name', mdl);
    chart = model.find('-isa', 'Stateflow.EMChart', 'Path', blkPath);
    chart.Script = code;
end
end
