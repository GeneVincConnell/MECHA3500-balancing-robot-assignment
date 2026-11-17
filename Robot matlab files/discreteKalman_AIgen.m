function kest = discreteKalman_AIgen(A, B, G, C, D, Qn, Rn, Ts)
%DISCRETEKALMAN_AIGEN  Steady-state discrete Kalman filter for a continuous
%plant, without any toolbox.
%
%   kest = discreteKalman_AIgen(A, B, G, C, D, Qn, Rn, Ts)
%   Plant:  dx/dt = A x + B u + G w,   y = C x + D u + v,
%           w: white process noise, intensity Qn;  v: measurement noise, Rn.
%   Same as the Control System Toolbox's
%           kest = kalmd(ss(A, [B G], C, [D 0]), Qn, Rn, Ts)
%   for what the Simulink model uses: the estimator state xhat[k|k-1] and
%       xhat[k+1] = kest.A*xhat[k] + kest.B*[u[k]; y[k]]
%   kest.A = Ad - L*C,  kest.B = [Bd - L*D, L], with L the predictor gain.
%   Also returns kest.L, kest.Ad, kest.Bd, kest.Qd, kest.Rd.
%
%   Like kalmd: zero-order-hold discretisation of A, B; Qd = int_0^Ts
%   expm(A t) G Qn G' expm(A' t) dt (Van Loan's method); Rd = Rn/Ts.
%
% AI-GENERATED (Claude, 2026-11-14). Written by Claude (Anthropic) for Gene
% Connell's MCHA3500 balancing robot so the estimator design runs without
% the Control System Toolbox. No code copied.

n = size(A, 1);
m = size(B, 2);
E  = expm([A, B; zeros(m, n + m)]*Ts);
Ad = E(1:n, 1:n);
Bd = E(1:n, n+1:end);
V  = expm([-A, G*Qn*G'; zeros(n), A']*Ts);       % Van Loan
Qd = V(n+1:end, n+1:end)'*V(1:n, n+1:end);
Qd = (Qd + Qd')/2;
Rd = Rn/Ts;
% Kalman filter = LQR of the dual system
Pp = solveDARE_AIgen(Ad', C', Qd, Rd);            % prediction covariance
L  = Ad*Pp*C'/(C*Pp*C' + Rd);                     % predictor gain
kest = struct('A', Ad - L*C, 'B', [Bd - L*D, L], 'L', L, ...
              'Ad', Ad, 'Bd', Bd, 'Qd', Qd, 'Rd', Rd);
end
