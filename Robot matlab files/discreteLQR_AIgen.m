function [K, Ad, Bd] = discreteLQR_AIgen(A, B, Q, R, Ts)
%DISCRETELQR_AIGEN  Discrete LQR from continuous weights, without any toolbox.
%
%   [K, Ad, Bd] = discreteLQR_AIgen(A, B, Q, R, Ts)
%   Same as the Control System Toolbox's  K = lqrd(A, B, Q, R, Ts):
%   the continuous plant dx/dt = A x + B u is discretised with a zero-order
%   hold at Ts, and the continuous cost  int(x'Qx + u'Ru) dt  is turned into
%   the exactly equivalent discrete cost (including the cross term it
%   creates). Then u[k] = -K x[k] minimises it.
%
%   Discretisation by Van Loan's method (C. F. Van Loan, "Computing integrals
%   involving the matrix exponential", IEEE TAC, 1978): with F = [A B; 0 0]
%   and W = blkdiag(Q, R),
%     E = expm([-F', W; 0, F]*Ts),  Phi = E22 = expm(F*Ts),
%     int_0^Ts expm(F't) W expm(F t) dt = E22' * E12.
%
% AI-GENERATED (Claude, 2026-11-14). Written by Claude (Anthropic) for Gene
% Connell's MCHA3500 balancing robot so the controller design runs without
% the Control System Toolbox. No code copied.

n = size(A, 1);
m = size(B, 2);
k = n + m;
F = [A, B; zeros(m, k)];
W = blkdiag(Q, R);
E = expm([-F', W; zeros(k), F]*Ts);
Phi = E(k+1:end, k+1:end);
Wd  = Phi'*E(1:k, k+1:end);
Wd  = (Wd + Wd')/2;
Ad = Phi(1:n, 1:n);
Bd = Phi(1:n, n+1:end);
Qd = Wd(1:n, 1:n);
Nd = Wd(1:n, n+1:end);
Rd = Wd(n+1:end, n+1:end);
[~, K] = solveDARE_AIgen(Ad, Bd, Qd, Rd, Nd);
end
