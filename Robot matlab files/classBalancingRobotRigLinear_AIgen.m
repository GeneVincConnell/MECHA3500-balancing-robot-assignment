classdef classBalancingRobotRigLinear_AIgen
    % AI-GENERATED (Claude, 2026-11-14)
    % Written by Claude (Anthropic) for Gene Connell's MCHA3500 balancing
    % robot. Not copied from any outside source. It is the flow-source
    % (Hamiltonian) model of classBalancingRobotFlowLinear_AIgen with the
    % wheels taken off and the axle held still by the test rig.
    %
    % classBalancingRobotRigLinear_AIgen  Linearised flow-source model of
    % the robot on the rig: no wheels, the motor shafts sit in the rig at a
    % fixed height, so the whole chassis pivots about the axle. No friction.
    %
    % Same coordinates as the wheeled model:
    %   state   x      = [phi; theta; ptheta]
    %   input   omega  = dphi, motor shaft speed relative to the chassis [rad/s]
    %   phi     angle between chassis and motor shafts (encoders)
    %   theta   chassis angle, clockwise from vertical
    %   ptheta  momentum in the theta coordinate, dL/d(dtheta)
    %
    % With the axle fixed, the "wheels" are only the two rotors and shafts:
    % in the wheeled model put r = 0, mw = 0 and Jw = Jr (both rotors). The
    % mass matrix becomes
    %   M = [Jr, Jr;  Jr, Jr + Jc + mc*l^2]
    % so, linearised about theta = 0 (CoM straight above the axle):
    %   dx = A*x + B*omega
    %   A  = [0, 0,      0;
    %         0, 0,      A0;
    %         0, mc*g*l, 0]
    %   B  = [1; -B0; 0]
    %   A0 = 1/(Jr + Jc + mc*l^2)      (1 / inertia about the axle)
    %   B0 = Jr*A0
    % With omega = 0 (motors holding, or unpowered with the rotors turning
    % with the chassis, as the encoders showed on 14 Nov) the free motion is
    % a frictionless inverted pendulum about the axle:
    %   theta_ddot = lambda^2*theta,  lambda^2 = A0*mc*g*l = g/L_eq,
    %   L_eq = (Jr + Jc + mc*l^2)/(mc*l)   (equivalent pendulum length)
    %
    % Usage:
    %   rig = classBalancingRobotRigLinear_AIgen;      % same estimates as
    %                                                  % the wheeled model
    %   rig.unstablePole()                             % lambda [1/s]
    %   [th, dth] = rig.freeResponse(t, theta0, dtheta0);

    properties
        mc          % Mass of the chassis (robot without wheels) [kg]
        Jc          % Moment of inertia of the chassis about its CoG, rotors excluded [kg m^2]
        Jr          % Both motor rotors and shafts about the axle [kg m^2]
        l           % Distance from the axle to the chassis CoG [m]
        g           % Acceleration due to gravity [m/s^2]
    end

    methods

        function obj = classBalancingRobotRigLinear_AIgen(p)
            % Parameters from classBalancingRobotFlowLinear_AIgen's
            % estimatedParameters(), so both models always agree. Pass a
            % struct p (fields mc, Jc, Jr, l, g) to use other values.
            if nargin < 1
                p = classBalancingRobotFlowLinear_AIgen.estimatedParameters();
            end
            obj.mc = p.mc;
            obj.Jc = p.Jc;
            obj.Jr = p.Jr;
            obj.l  = p.l;
            obj.g  = p.g;
        end

        function J = Jaxle(obj)
            % Inertia of everything that turns about the axle [kg m^2].
            J = obj.Jr + obj.Jc + obj.mc*obj.l^2;
        end

        function A0 = Acoef(obj)
            % A(theta) at theta = 0: 1/M22.
            A0 = 1/obj.Jaxle();
        end

        function B0 = Bcoef(obj)
            % B(theta) at theta = 0: M21/M22.
            B0 = obj.Jr*obj.Acoef();
        end

        function A_lin = A(obj)
            % A matrix of the linearised flow model, x = [phi; theta; ptheta].
            A_lin = [0, 0,                  0;
                     0, 0,                  obj.Acoef();
                     0, obj.mc*obj.g*obj.l, 0];
        end

        function B_lin = B(obj)
            % B matrix of the linearised flow model, input omega = dphi.
            B_lin = [1; -obj.Bcoef(); 0];
        end

        function dx = state_derivative(obj, x, omega)
            % Time derivative of x = [phi; theta; ptheta], linearised model.
            dx = obj.A()*x(:) + obj.B()*omega;
        end

        function ptheta = momentum(obj, dphi, dtheta)
            % ptheta from measured rates: (B0*dphi + dtheta)/A0.
            ptheta = (obj.Bcoef()*dphi + dtheta)/obj.Acoef();
        end

        function lambda = unstablePole(obj)
            % Unstable eigenvalue of the linearised model [1/s].
            lambda = max(real(eig(obj.A())));
        end

        function L = equivalentLength(obj)
            % Length of the simple pendulum that falls at the same rate:
            % L_eq = g/lambda^2 = Jaxle/(mc*l) [m].
            L = obj.Jaxle()/(obj.mc*obj.l);
        end

        function [theta, dtheta] = freeResponse(obj, t, theta0, dtheta0)
            % Linear model's fall with omega = 0 from theta0 [rad] and
            % dtheta0 [rad/s] at t = 0: x(t) = expm(A*t)*x0.
            % theta is measured from the balance point (CoM above the axle).
            Am = obj.A();
            x0 = [0; theta0; obj.momentum(0, dtheta0)];
            theta  = zeros(size(t));
            dtheta = zeros(size(t));
            for k = 1:numel(t)
                x = expm(Am*t(k))*x0;
                theta(k)  = x(2);
                dtheta(k) = obj.Acoef()*x(3);      % dtheta = A0*ptheta - B0*omega, omega = 0
            end
        end

    end
end
