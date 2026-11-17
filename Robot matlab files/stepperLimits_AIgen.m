function lim = stepperLimits_AIgen(robot, motor)
% AI-GENERATED (Claude, 2026-11-15)
% Written by Claude (Anthropic) for Gene Connell's MCHA3500 balancing robot.
% Not copied from any outside source. The method is MCHA3500 Assignment 2,
% Problem 1, Task 1.1 (feasible velocities and accelerations and the
% performance metric A_p), with the DC motor of parts c)-f) replaced by the
% stepper's pull-out torque curve.
%
% stepperLimits_AIgen  Wheel speed and acceleration limits for the
% controller, from the robot parameters and the stepper pull-out curve.
%
%   lim = stepperLimits_AIgen(robot, motor)
%
% robot: anything with fields/properties mc, mw, Jw, l, r, g
%        (classBalancingRobotFlowLinear_AIgen works).
% motor: struct with
%   wCurve, tauCurve  pull-out curve of ONE motor: speed [rad/s], torque [N m]
%   Vcurve, Icurve    supply voltage [V] and phase current [A] of that curve
%   curveIsRms        true if Icurve is an RMS value (true for most stepper
%                     datasheets), false if it is a peak value
%   Vbatt             supply voltage on the robot [V]
%   IrunPeak          driver run current, peak of the sine [A] (TMC2240 IRUN)
%   SF                safety factor on torque (1 = the pull-out curve itself)
%   omegaMaxUser      NaN: choose the limits that maximise A_p;
%                     a number: use that speed limit [rad/s] and the
%                     acceleration available up to it
%
% The steps, following Assignment 2 Task 1.1 (flat ground, no friction,
% q = [phi; theta], phi = wheel angle relative to the chassis):
%   a)  M(q)*ddq + C*dq + g(q) = [T; 0], with
%       M = [a, a + c*cos(theta); a + c*cos(theta), a + b + 2*c*cos(theta)],
%       a = Jw + (mw + mc)*r^2,  b = Jc + mc*l^2,  c = mc*l*r,
%       g(q) = [0; -mc*g*l*sin(theta)],  T = torque of both motors.
%   b)  Holding a constant lean theta* (dtheta = ddtheta = 0):
%         row 2:  ddphi = mc*g*l*sin(theta*)/(a + c*cos(theta*))
%         row 1:  T = a*ddphi = f(theta*) = a*mc*g*l*sin(theta*)/(a + c*cos(theta*))
%   c)  Stepper instead of the DC motor: the motors can give any torque up to
%       the pull-out torque at the current speed, |T| <= Tmax(dphi), where
%       Tmax = 2 motors x pull-out(dphi) x current and voltage scaling / SF.
%   d)  Largest lean the motors can hold at speed dphi: f(theta_max) = Tmax(dphi)
%       (f is increasing on -pi/2..pi/2, so there is one solution).
%   e)  Upright is feasible (theta_min <= 0 <= theta_max) wherever
%       Tmax(dphi) >= 0, i.e. over the whole measured curve: the stepper has
%       no back-EMF ceiling like the DC motor, only a shrinking margin.
%   f)  Feasible wheel accelerations at speed dphi: |ddphi| <= Tmax(dphi)/a.
%   g)  A_p = (ds_max - ds_min)*(dds_max - dds_min), s = r*phi. With the DC
%       motor this region is a rectangle. With the stepper the acceleration
%       bound shrinks with speed, so the controller's box limits
%       (|omega| <= omegaMax, |alpha| <= alphaMax) are the largest-area
%       rectangle that fits inside the feasible region:
%         alphaMax(omegaMax) = min over |dphi| <= omegaMax of Tmax(dphi)/a
%         A_p = (2*r*omegaMax)*(2*r*alphaMax), maximised over omegaMax.
%
% Scaling of the datasheet curve (approximate, replace with a measured
% curve when you have one):
%   current: torque proportional to current, capped at the curve's current;
%            applied at all speeds (conservative at high speed, where the
%            driver cannot reach the set current anyway);
%   voltage: the high-speed roll-off is voltage limited, so the speed axis
%            is stretched by Vbatt/Vcurve.

%% Robot
a = robot.Jw + (robot.mw + robot.mc)*robot.r^2;   % rigid-robot inertia at the axle [kg m^2]
c = robot.mc*robot.l*robot.r;
mgl = robot.mc*robot.g*robot.l;

%% Pull-out torque of one motor on this robot
if motor.curveIsRms
    IcurvePeak = sqrt(2)*motor.Icurve;            % sine: peak = sqrt(2) x RMS
else
    IcurvePeak = motor.Icurve;
end
kI = min(1, motor.IrunPeak/IcurvePeak);           % current scaling
kV = motor.Vbatt/motor.Vcurve;                    % voltage scaling of the speed axis
wTab   = [0, motor.wCurve(:).'*kV];
tauTab = [motor.tauCurve(1), motor.tauCurve(:).'];   % flat below the first point

w      = linspace(0, wTab(end), 4001);             % wheel speed grid [rad/s]
tauPO  = kI*interp1(wTab, tauTab, w);              % pull-out torque per motor [N m]
tauUse = tauPO/motor.SF;                           % design torque per motor [N m]

%% Task 1.1 f): acceleration bound, both motors
alphaCap = 2*tauUse/a;                             % [rad/s^2]
alphaBox = cummin(alphaCap);                       % largest acceleration available at
                                                   % every speed up to w

%% Task 1.1 g): largest-area box inside the feasible region
Ap = (2*robot.r*w).*(2*robot.r*alphaBox);          % [m/s * m/s^2]
if isnan(motor.omegaMaxUser)
    [ApMax, k] = max(Ap);
else
    [~, k] = min(abs(w - motor.omegaMaxUser));
    ApMax = Ap(k);
end
omegaMax = w(k);
alphaMax = alphaBox(k);

%% Task 1.1 b), d): lean needed to sustain alphaMax (= largest lean the
% design torque can hold at omegaMax)
T = a*alphaMax;                                    % both motors [N m]
f = @(th) a*mgl*sin(th)./(a + c*cos(th));
if T >= f(pi/2)
    thetaHold = pi/2;
else
    thetaHold = fzero(@(th) f(th) - T, [0, pi/2]);
end

%% Outputs
lim.a            = a;
lim.w            = w;              % speed grid [rad/s]
lim.tauPO        = tauPO;          % pull-out torque per motor on the grid [N m] (steps are lost above it)
lim.tauUse       = tauUse;         % pull-out / SF [N m]
lim.alphaCap     = alphaCap;       % Task 1.1 f) bound at each speed, with SF [rad/s^2]
lim.omegaUpright = wTab(end);      % Task 1.1 e): upright feasible up to the end of the curve [rad/s]
lim.omegaMax     = omegaMax;       % [rad/s]
lim.alphaMax     = alphaMax;       % [rad/s^2]
lim.tauAvail     = a*alphaMax/2;   % torque per motor that alphaMax needs [N m]
lim.Ap           = ApMax;          % [m^2/s^3]
lim.thetaHold    = thetaHold;      % lean that sustains alphaMax [rad]
lim.kI           = kI;
lim.kV           = kV;
end
