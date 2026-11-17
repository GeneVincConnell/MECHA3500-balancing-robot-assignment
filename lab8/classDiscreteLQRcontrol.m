classdef classDiscreteLQRcontrol < handle
    % classDiscreteLQRcontrol  Discrete-time LQR controller (MCHA3500 Lab 8).
    %
    %   Handle class, so gains and the integrator state are stored on the
    %   object when a method changes them.
    %
    %   Regulation:  u = -K*x
    %   Integrator:  u = -K*[x; z],  z(k+1) = A_z*[x; z] + B_z*u
    %                (z is the Euler-discretised integral of y = C_r*x + D_r*u)

    properties
        controlMode = 'None'    % Indicator of active control mode
        T                       % Sampling period of controller
        A                       % Linearised A matrix for controller
        B                       % Linearised B matrix for controller
        C_r                     % Linearised C matrix for regulation output
        D_r                     % Linearised D matrix for regulation output
        A_int                   % A matrix augmented with integrator
        B_int                   % B matrix augmented with integrator
        Q                       % Q matrix for LQR cost
        R                       % R matrix for LQR cost
        N                       % N matrix for LQR cost
        K                       % Optimal control gain
        N_x                     % Feed-forward gain for states
        N_u                     % Feed-forward gain for inputs
        y_ref                   % Output regulation reference
        A_z                     % State coefficient for integrator transition dynamics
        B_z                     % Input coefficient for integrator transition dynamics
        z                       % Integrator state
    end

    methods
        function controlInit_regulation(obj, A, B, Q, R, N, T)
            % State-feedback LQR. Q, R, N are continuous-time weights;
            % lqrd converts them to the equivalent discrete-time design.
            obj.A = A;
            obj.B = B;
            obj.Q = Q;
            obj.R = R;
            obj.N = N;
            obj.T = T;

            % Compute optimal control gain
            obj.K = lqrd(A, B, Q, R, N, T);

            % Update indicator
            obj.controlMode = 'Regulation';
        end

        function controlInit_integrator(obj, A, B, C_r, D_r, Q, R, N, T)
            % LQR with integral action on the regulated output y = C_r*x + D_r*u
            % Store properties
            obj.A = A;
            obj.B = B;
            obj.C_r = C_r;
            obj.D_r = D_r;
            obj.Q = Q;
            obj.R = R;
            obj.N = N;
            obj.T = T;

            % Construct continuous-time augmented A B matrices with integrator
            % d/dt [x; z] = [A 0; C_r 0]*[x; z] + [B; D_r]*u
            n = size(A, 1);
            p = size(C_r, 1);
            obj.A_int = [A,   zeros(n, p);
                         C_r, zeros(p, p)];
            obj.B_int = [B;
                         D_r];

            % Define the discrete-time integrator update matrices using an Euler
            % discretisation. ie. Matrices that satisfy:
            % z_(k+1) = A_z*[x_k-x_star; z_k] + B_z*u_k
            % From z_(k+1) = z_k + T*(C_r*x_k + D_r*u_k)
            obj.A_z = [T*C_r, eye(p)];
            obj.B_z = T*D_r;

            % Compute optimal control gain
            obj.K = lqrd(obj.A_int, obj.B_int, Q, R, N, T);

            % Initialise integrator
            obj.z = zeros(p, 1);

            % Update indicator
            obj.controlMode = 'Integrator';
        end

        function u = control(obj, x)
            % Switch between control modes
            switch obj.controlMode
                % Regulation only control
                case 'Regulation'
                    u = -obj.K*x;

                % Integral control
                case 'Integrator'
                    % Concatenate state vector with integrator state
                    state = [x; obj.z];
                    % Compute control signal
                    u = -obj.K*state;
                    % Update integrator state for next iteration
                    obj.z = obj.A_z*state + obj.B_z*u;

                % Default case if control is un-initialised
                otherwise
                    u = 0;
            end
        end
    end
end
