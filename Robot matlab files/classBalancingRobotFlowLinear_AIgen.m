classdef classBalancingRobotFlowLinear_AIgen
    % AI-GENERATED (Claude, 2026-11-14)
    % Written by Claude (Anthropic) for Gene Connell's MCHA3500 balancing
    % robot. Not copied from any outside source. The model follows the
    % flow-source (Hamiltonian) formulation of MCHA3500 Assignment 1,
    % Problem 2, re-derived symbolically on 2026-11-14.
    %
    % classBalancingRobotFlowLinear_AIgen  Linearised flow-source model of
    % the stepper-driven balancing robot, with parameters estimated from
    % measurements of the real robot.
    %
    % Same interface as classBalancingRobotFlowLinear (Assignment 1, Task 2.3):
    %   state   x      = [phi; theta; ptheta]
    %   input   omega  = dphi, wheel speed relative to the chassis [rad/s]
    %   phi     angle between chassis and wheel (what the steppers and
    %           encoders act on / measure)
    %   theta   chassis angle, clockwise from vertical
    %   ptheta  momentum in the theta coordinate, dL/d(dtheta)
    %
    % Linearised about x = 0, omega = 0, alpha = 0 (flat ground):
    %   dx = A*x + B*omega
    %   A  = [0, 0,      0;
    %         0, 0,      A0;
    %         0, mc*g*l, -b_theta*A0]
    %   B  = [1; -B0; b_theta*B0]
    % where A0 and B0 are A(theta) and B(theta) of
    %   thetaDot = A(theta)*ptheta - B(theta)*omega
    % evaluated at theta = 0:
    %   A0 = 1/(Jw + Jc + mw*r^2 + mc*(r + l)^2)
    %   B0 = (Jw + mw*r^2 + mc*r*(r + l))*A0
    %
    % With omega = 0 the steppers hold the wheels locked to the chassis, so
    % the free motion is the whole robot rolling as one rigid body. The
    % unstable pole is sqrt(A0*mc*g*l).
    %
    % Usage:
    %   robot = classBalancingRobotFlowLinear_AIgen;   % estimated parameters
    %   robot.l = 0.06;                                % override any value
    %   A = robot.A();  B = robot.B();
    %   eig(A)                                         % 0 and +/- pole
    %
    % Which parameters are estimates and which are assumptions is recorded
    % in estimatedParameters() below. Replace the assumptions with
    % measurements as you get them.

    properties
        mc          % Mass of the chassis [kg] (everything except the wheels)
        Jc          % Moment of inertia of the chassis about its CoG [kg m^2]
        mw          % Combined mass of both wheels [kg]
        Jw          % Combined inertia of both wheels and motor rotors about the axle [kg m^2]
        l           % Distance from the axle to the chassis CoG [m]
        r           % Radius of the wheel [m]
        g           % Acceleration due to gravity [m/s^2]
        b_phi       % Damping coefficient in the phi coordinate [N m s/rad] (unused here: phi is driven directly)
        b_theta     % Damping coefficient in the theta coordinate [N m s/rad]
    end

    methods

        function obj = classBalancingRobotFlowLinear_AIgen()
            % Load the estimated parameters of the real robot.
            p = classBalancingRobotFlowLinear_AIgen.estimatedParameters();
            obj.mc      = p.mc;
            obj.Jc      = p.Jc;
            obj.mw      = p.mw;
            obj.Jw      = p.Jw;
            obj.l       = p.l;
            obj.r       = p.r;
            obj.g       = p.g;
            obj.b_phi   = p.b_phi;
            obj.b_theta = p.b_theta;
        end

        function A0 = Acoef(obj)
            % A(theta) at theta = 0, alpha = 0: 1/M22.
            A0 = 1/(obj.Jw + obj.Jc + obj.mw*obj.r^2 + obj.mc*(obj.r + obj.l)^2);
        end

        function B0 = Bcoef(obj)
            % B(theta) at theta = 0, alpha = 0: M21/M22.
            B0 = (obj.Jw + obj.mw*obj.r^2 + obj.mc*obj.r*(obj.r + obj.l))*obj.Acoef();
        end

        function A_lin = A(obj)
            % A matrix of the linearised flow model, x = [phi; theta; ptheta].
            A0 = obj.Acoef();
            A_lin = [0, 0,                    0;
                     0, 0,                    A0;
                     0, obj.mc*obj.g*obj.l,   -obj.b_theta*A0];
        end

        function B_lin = B(obj)
            % B matrix of the linearised flow model, input omega = dphi.
            B0 = obj.Bcoef();
            B_lin = [1;
                     -B0;
                     obj.b_theta*B0];
        end

        function dx = state_derivative(obj, x, omega)
            % Time derivative of x = [phi; theta; ptheta] according to the
            % linearised model.
            dx = obj.A()*x(:) + obj.B()*omega;
        end

        function ptheta = momentum(obj, dphi, dtheta)
            % ptheta from measured rates (encoder dphi, gyro dtheta), for
            % setting initial conditions or estimating the state:
            % ptheta = M21*dphi + M22*dtheta = (B0*dphi + dtheta)/A0.
            ptheta = (obj.Bcoef()*dphi + dtheta)/obj.Acoef();
        end

        function C_out = C_meas(obj)
            % Output matrix for y = [phi; theta; dtheta]: encoder, IMU angle
            % and gyro rate. dtheta = A0*ptheta - B0*omega.
            C_out = [1, 0, 0;
                     0, 1, 0;
                     0, 0, obj.Acoef()];
        end

        function D_out = D_meas(obj)
            % Feedthrough matrix for y = [phi; theta; dtheta].
            D_out = [0; 0; -obj.Bcoef()];
        end

        function lambda = unstablePole(obj)
            % Unstable eigenvalue of the linearised model [1/s].
            lambda = max(real(eig(obj.A())));
        end

    end

    methods (Static)

        function p = estimatedParameters(yRestMid)
            % Parameter estimates and where each number comes from.
            % Optional yRestMid [m]: height above the axle of the middle of
            % the remaining chassis mass (default 0.08, the assumption
            % below). fitRigFall_AIgen.m sweeps it to show how the CoM
            % height l and the inertia change together.
            if nargin < 1
                yRestMid = 0.08;
            end

            % --- Measured by Gene (14 Nov 2026) ---
            mc     = 2.1088;           % robot without wheels [kg]
            depth  = 0.16;             % chassis front-to-back depth [m]
            yBatLo = 0.16;             % battery bottom, height above the axle [m]
            yBatHi = 0.20;             % battery top (top of the robot) [m]
            r      = 0.052;            % wheel radius, about 5.2 cm [m]
            mWheel = 0.138;            % one wheel including hub [kg]

            % --- Datasheets ---
            mMotor = 0.556;            % 17HS24-2004D-E1K, StepperOnline product
                                       % page (the datasheet drawing says 0.65 kg)
            aMotor = 0.042;            % NEMA 17 frame, 42 mm square, coaxial with the axle
            Jrotor = 90e-7;            % rotor inertia 90 g cm^2 per motor (datasheet)
            mBat   = 0.349;            % CNHL G+Plus 2200 mAh 6S, 349 g (CNHL spec)
            batH   = 0.035;            % battery 35 x 51 x 109 mm; 35 mm assumed vertical
            batD   = 0.051;            % and 51 mm front-to-back

            % --- Assumptions: replace with measurements ---
            yRestLo = yRestMid - 0.08; % the remaining chassis mass (frame, PCB,
            yRestHi = yRestMid + 0.08; % drivers, wiring) is spread evenly over
                                       % 16 cm: by default from the axle up to
                                       % the battery

            % --- Chassis CoG height and inertia, tallied about the axle ---
            mRest = mc - 2*mMotor - mBat;
            yBat  = (yBatLo + yBatHi)/2;
            yRest = (yRestLo + yRestHi)/2;
            hRest = yRestHi - yRestLo;

            l = (mBat*yBat + mRest*yRest)/mc;          % motors contribute at y = 0

            JaxleMotors = 2*mMotor*(2*aMotor^2)/12 - 2*Jrotor;  % rotors spin with the wheels
            JaxleBat    = mBat*(yBat^2 + (batH^2 + batD^2)/12);
            JaxleRest   = mRest*(yRest^2 + (hRest^2 + depth^2)/12);
            Jc = JaxleMotors + JaxleBat + JaxleRest - mc*l^2;   % parallel-axis shift to the CoG

            % --- Wheels: both wheels as uniform discs, plus both rotors ---
            mw = 2*mWheel;
            Jw = 2*(0.5*mWheel*r^2) + 2*Jrotor;

            p = struct('mc', mc, 'Jc', Jc, 'mw', mw, 'Jw', Jw, 'l', l, 'r', r, ...
                       'g', 9.81, ...
                       'Jr', 2*Jrotor, ...     % both motor rotors, about the axle
                       'b_phi', 0, ...         % unknown, set to zero
                       'b_theta', 0);          % unknown, set to zero
        end

    end
end
