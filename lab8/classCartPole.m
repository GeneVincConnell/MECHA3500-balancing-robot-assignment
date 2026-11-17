classdef classCartPole
    % classCartPole  Cart-pole on an inclined plane (MCHA3500 Lab 2).
    %
    %   q  = [s; theta]          generalised coordinates
    %   dq = [ds; dtheta]        generalised velocities
    %   x  = [s; theta; ds; dtheta]
    %
    %   s     : cart position along the incline
    %   theta : pendulum angle measured clockwise from the cart's normal
    %           (theta = alpha when the pendulum is vertical)
    %   alpha : ground angle
    %   F     : force on the cart along the incline
    %
    %   Equations of motion:  M(q)*ddq + C(q,dq)*dq + D*dq + grav(q,alpha) = [F; 0]

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
        function massMat = M(obj, q)
            % Mass matrix M(q)
            theta = q(2);
            massMat = [obj.mc + obj.mp,          obj.mp*obj.l*cos(theta);
                       obj.mp*obj.l*cos(theta),  obj.mp*obj.l^2];
        end

        function corMat = C(obj, q, dq)
            % Centripetal-Coriolis matrix C(q,dq), from C*dq = dM/dt*dq - dT/dq
            theta  = q(2);
            dtheta = dq(2);
            corMat = [0, -obj.mp*obj.l*sin(theta)*dtheta;
                      0,  0];
        end

        function potentialGrad = grav(obj, q, alpha)
            % Gradient of the potential energy, g(q) = dV/dq
            theta = q(2);
            potentialGrad = [(obj.mc + obj.mp)*obj.g*sin(alpha);
                             -obj.mp*obj.g*obj.l*sin(theta - alpha)];
        end

        function dampingMat = D(obj)
            % Damping matrix
            dampingMat = diag([obj.b_s, obj.b_theta]);
        end

        function kineticCoEng = T(obj, q, dq)
            % Kinetic co-energy T*(q,dq) = 1/2 dq' M(q) dq
            kineticCoEng = 0.5*dq.'*obj.M(q)*dq;
        end

        function potentialEng = V(obj, q, alpha)
            % Potential energy (zero at s = 0 with the pendulum horizontal)
            s     = q(1);
            theta = q(2);
            potentialEng = (obj.mc + obj.mp)*obj.g*s*sin(alpha) ...
                         + obj.mp*obj.g*obj.l*cos(theta - alpha);
        end

        function dx = state_derivative(obj, x, F, alpha)
            % Time derivative of the state x = [s; theta; ds; dtheta]
            q   = x(1:2);
            dq  = x(3:4);
            tau = [F; 0];
            ddq = obj.M(q) \ (tau - obj.C(q,dq)*dq - obj.D()*dq - obj.grav(q,alpha));
            dx  = [dq; ddq];
        end
    end
end
