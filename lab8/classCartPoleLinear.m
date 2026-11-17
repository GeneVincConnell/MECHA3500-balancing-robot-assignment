classdef classCartPoleLinear
    % classCartPoleLinear  Cart-pole linearised about the upright equilibrium
    % (s, theta, ds, dtheta) = 0 with alpha = 0 (MCHA3500 Lab 8, Problem 1).
    %
    %   Second-order form:  M*ddq + D*dq + K*q = G*F
    %   First-order form:   dx = A*x + B*F,   x = [s; theta; ds; dtheta]

    properties
        mc          % Mass of the cart
        mp          % Mass of the pendulum
        l           % Length of the pendulum
        r           % Radius of the wheel (not used in the force-input model)
        g           % Acceleration due to gravity
        b_s         % Damping coefficient in the s coordinate
        b_theta     % Damping coefficient in the theta coordinate
    end

    methods
        function massMatLin = M(obj)
            % Mass matrix evaluated at theta = 0
            massMatLin = [obj.mc + obj.mp,  obj.mp*obj.l;
                          obj.mp*obj.l,     obj.mp*obj.l^2];
        end

        function dampingMatLin = D(obj)
            % Damping matrix (already linear)
            dampingMatLin = diag([obj.b_s, obj.b_theta]);
        end

        function potGradLin = K(obj)
            % Jacobian of the potential gradient g(q) at q = 0, alpha = 0
            potGradLin = [0, 0;
                          0, -obj.mp*obj.g*obj.l];
        end

        function inputMatLin = G(obj)
            % Input mapping: tau = G*F (force acts on the s coordinate only)
            inputMatLin = [1; 0];
        end

        function A_lin = A(obj)
            % State matrix for x = [q; v], v = dq
            Minv = obj.M() \ eye(2);
            A_lin = [zeros(2),        eye(2);
                     -Minv*obj.K(),   -Minv*obj.D()];
        end

        function B_lin = B(obj)
            % Input matrix for x = [q; v]
            B_lin = [zeros(2,1);
                     obj.M() \ obj.G()];
        end

        function dx = state_derivative(obj, x, F)
            % Time derivative of the state according to the linearised model
            dx = obj.A()*x + obj.B()*F;
        end
    end
end
