function [P, K] = solveDARE_AIgen(A, B, Q, R, N)
%SOLVEDARE_AIGEN  Discrete algebraic Riccati equation without any toolbox.
%
%   [P, K] = solveDARE_AIgen(A, B, Q, R)
%   [P, K] = solveDARE_AIgen(A, B, Q, R, N)
%   Solves  P = A'PA - (A'PB + N)(R + B'PB)^-1 (B'PA + N') + Q
%   and returns the LQR gain K = (R + B'PB)^-1 (B'PA + N'), so that
%   u[k] = -K x[k] minimises sum(x'Qx + u'Ru + 2x'Nu). Same results as the
%   Control System Toolbox's dlqr / idare.
%
%   Method: the cross term is removed (A - B R^-1 N', Q - N R^-1 N'), then
%   the structure-preserving doubling algorithm (Chu, Fan and Lin, 2005,
%   "A structure-preserving doubling algorithm for continuous-time algebraic
%   Riccati equations" / its discrete-time form), which converges
%   quadratically when a stabilising solution exists.
%
% AI-GENERATED (Claude, 2026-11-14). Written by Claude (Anthropic) for Gene
% Connell's MCHA3500 balancing robot so the controller design runs without
% the Control System Toolbox. The algorithm is the published SDA; no code
% was copied.

n = size(A, 1);
if nargin < 5 || isempty(N)
    N = zeros(n, size(B, 2));
end
Ri = R \ eye(size(R));
Ak = A - B*Ri*N';                  % remove the cross term
Hk = Q - N*Ri*N';
Hk = (Hk + Hk')/2;
Gk = B*Ri*B';
Gk = (Gk + Gk')/2;
I  = eye(n);
for it = 1:200
    W   = I + Gk*Hk;
    WA  = W \ Ak;                  % (I + G H)^-1 A
    WG  = W \ Gk;                  % (I + G H)^-1 G
    Hn  = Hk + Ak'*Hk*WA;
    Gn  = Gk + Ak*WG*Ak';
    Ak  = Ak*WA;
    Hn  = (Hn + Hn')/2;
    Gk  = (Gn + Gn')/2;
    done = norm(Hn - Hk, 1) <= 1e-13*max(1, norm(Hn, 1));
    Hk  = Hn;
    if done
        break
    end
end
if ~done
    error('solveDARE_AIgen:noConvergence', ...
        'The Riccati iteration did not converge: is (A, B) stabilisable and (A, Q) detectable?');
end
P = Hk;
K = (R + B'*P*B) \ (B'*P*A + N');
end
