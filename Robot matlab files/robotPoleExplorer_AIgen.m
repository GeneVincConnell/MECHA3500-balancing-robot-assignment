%% robotPoleExplorer_AIgen.m
%  Balancing robot - where does a given build land on the lambda_3 curves,
%  and where do its poles sit?
%
%  Model: linearised FLOW (Hamiltonian) model about the upright equilibrium,
%         states x = [phi; theta; ptheta], input omega = phidot.
%             A = [ 0      0        0      ;
%                   0      0        A0     ;
%                   0   mc*g*l  -b_theta*A0 ]
%             B = [ 1; -B0; b_theta*B0 ]
%         A0 = 1/D,  B0 = (Jw + mw*r^2 + mc*r*(r+l))/D
%         D  = Jw + Jc + mw*r^2 + mc*(r+l)^2          (= M22 of the effort model)
%  Open-loop poles: 0 (wheel position integrator) and the roots of
%             s^2 + b_theta*A0*s - A0*mc*g*l = 0
%  With b_theta = 0 these are +/- lambda_3, lambda_3 = sqrt(mc*g*l/D).
%
%  AI-generated (Claude), 15 Nov 2026. Edit the USER INPUTS block and run.

clear variables; close all; clc;

%% =============================== USER INPUTS ===============================
P.mc      = 2.1088;     % chassis mass, wheels NOT included          [kg]
P.l       = 0.0544;     % chassis CoG height above the wheel axle    [m]
P.r       = 0.0520;     % wheel radius                               [m]
P.mWheel  = 0.138;      % ONE wheel including its hub                [kg]
P.Jrotor  = 90e-7;      % rotor inertia of ONE motor                 [kg m^2]
P.b_theta = 0;          % viscous damping on the lean axis           [N m s/rad]
P.g       = 9.81;       % gravity                                    [m/s^2]

% --- chassis inertia about its own CoG -------------------------------------
%  'rig'    : back it out of a measured topple (wheels off, pivot on the axle)
%  'direct' : type the number in yourself
P.JcMode    = 'rig';
P.lambdaRig = 7.60;     % fitted unstable pole of the rig fall       [1/s]
P.JcDirect  = 0.0132;   % used only when JcMode = 'direct'           [kg m^2]

% --- the mass you would physically move to retune l ------------------------
%  Moving it changes l AND Jc together, which is the only honest way to sweep
%  CoG height. Set mMove = 0 to freeze Jc instead.
P.mMove = 0.349;        % battery mass                               [kg]
P.yMove = 0.18;         % its height above the axle right now        [m]

% --- sweep ranges ----------------------------------------------------------
S.rRange     = [0.015 0.160];   % wheel radius sweep                 [m]
S.yRange     = [0.000 0.420];   % battery height sweep               [m]
S.wheelScale = true;            % also draw the curve where wheel mass ~ r^2
S.Kmax       = 25;              % lean-feedback gain range for the root locus
%% ============================= END USER INPUTS =============================


%% ------------------------- derived quantities ------------------------------
P  = derive(P);
me = model(P, P.r, P.l, P.Jc);

fprintf('\n=============== BALANCING ROBOT POLE EXPLORER ===============\n');
fprintf('  INPUTS    mc = %.4f kg    l = %.1f mm    r = %.1f mm\n', ...
        P.mc, P.l*1e3, P.r*1e3);
if strcmpi(P.JcMode,'rig')
    fprintf('  DERIVED   Jc = %.6f kg m^2   (from the rig fall, lambda = %.2f)\n', ...
            P.Jc, P.lambdaRig);
    fprintf('            for comparison, 0.5*mc*l^2 = %.6f  (%.1fx smaller)\n', ...
            0.5*P.mc*P.l^2, P.Jc/(0.5*P.mc*P.l^2));
else
    fprintf('  DERIVED   Jc = %.6f kg m^2   (entered directly)\n', P.Jc);
end
fprintf('            Jw = %.6f kg m^2   mw = %.4f kg\n', me.Jw, P.mw);
fprintf('            D  = %.6f kg m^2   A0 = %.4f   B0 = %.4f\n', me.D, me.A0, me.B0);
fprintf('  POLES     %+.4f   %+.4f   %+.4f   [1/s]\n', sort(real(me.poles),'descend'));
fprintf('  lambda_3  = %.4f 1/s   ->   doubling time %.1f ms\n', ...
        me.lambda, 1e3*log(2)/me.lambda);
if strcmpi(P.JcMode,'rig')
    fprintf('            the rig fell at %.2f 1/s; the wheels slow it by %.0f%%\n', ...
            P.lambdaRig, 100*(1 - me.lambda/P.lambdaRig));
end
fprintf('  CONTROL   ctrb rank = %d of 3\n', rank([me.B, me.A*me.B, me.A^2*me.B]));

%% ------------------------------ sweeps -------------------------------------
rr = linspace(S.rRange(1), S.rRange(2), 400);
lamR = zeros(size(rr));  lamRs = zeros(size(rr));
for k = 1:numel(rr)
    lamR(k)  = model(P, rr(k), P.l, P.Jc).lambda;                 % same wheel mass
    Pk = P;  Pk.mWheel = P.mWheel*(rr(k)/P.r)^2;  Pk.mw = 2*Pk.mWheel;
    lamRs(k) = model(Pk, rr(k), P.l, P.Jc).lambda;                % wheel mass ~ r^2
end

yy = linspace(S.yRange(1), S.yRange(2), 400);
lamY = zeros(size(yy));  lY = zeros(size(yy));
for k = 1:numel(yy)
    [lk, Jck] = moveMass(P, yy(k));
    lY(k)   = lk;
    lamY(k) = model(P, P.r, lk, Jck).lambda;
end
[lamWorst, iw] = max(lamY);
fprintf('  WORST     battery at %.0f mm  ->  l = %.1f mm, lambda_3 = %.3f 1/s\n', ...
        yy(iw)*1e3, lY(iw)*1e3, lamWorst);
if P.mMove > 0
    if P.yMove > yy(iw)
        fprintf('            you sit ABOVE it: raising the battery keeps helping\n');
    else
        fprintf('            you sit BELOW it: lowering the battery helps, raising hurts\n');
    end
end
fprintf('=============================================================\n\n');

%% ============================== FIGURE 1 ===================================
BLUE = [0.165 0.471 0.839];  ORANGE = [0.922 0.408 0.204];
GREY = [0.541 0.533 0.514];  INK   = [0.10 0.10 0.09];

figure('Name','Where this build lands','Color','w','Position',[60 60 1150 780]);

% --- (1) lambda_3 vs wheel radius -----------------------------------------
subplot(2,2,1); hold on; grid on; box off;
plot(rr*1e3, lamR, 'Color', BLUE, 'LineWidth', 2);
if S.wheelScale
    plot(rr*1e3, lamRs, '--', 'Color', ORANGE, 'LineWidth', 1.5);
    legend({'wheel mass held constant','wheel mass \propto r^2'}, ...
           'Location','southwest','Box','off');
end
plot(P.r*1e3, me.lambda, 's', 'MarkerSize', 10, 'MarkerFaceColor', INK, ...
     'MarkerEdgeColor','w','LineWidth',1.2);
text(P.r*1e3 + 6, me.lambda + 0.45, sprintf('r = %.0f mm\n\\lambda_3 = %.2f', ...
     P.r*1e3, me.lambda), 'FontWeight','bold','FontSize',9);
if strcmpi(P.JcMode,'rig')
    plot(S.rRange*1e3, [P.lambdaRig P.lambdaRig], ':', 'Color', GREY, 'LineWidth', 1);
    text(S.rRange(2)*1e3, P.lambdaRig+0.18, sprintf('rig, wheels off (%.2f)', ...
         P.lambdaRig), 'Color', GREY, 'FontSize', 8, 'HorizontalAlignment','right');
end
xlabel('wheel radius  r  [mm]'); ylabel('\lambda_3  [1/s]');
title('Unstable pole vs wheel radius');
xlim(S.rRange*1e3);
yTop = max([lamR(:); lamRs(:); me.lambda]);
if strcmpi(P.JcMode,'rig'), yTop = max(yTop, P.lambdaRig); end
ylim([0.92*min([lamR(:); lamRs(:)]) 1.10*yTop]);

% --- (2) lambda_3 vs CoG height -------------------------------------------
subplot(2,2,2); hold on; grid on; box off;
plot(lY*1e3, lamY, 'Color', BLUE, 'LineWidth', 2);
plot(lY(iw)*1e3, lamWorst, 'o', 'MarkerSize', 8, 'MarkerEdgeColor', BLUE, ...
     'MarkerFaceColor','w','LineWidth',1.8);
text(lY(iw)*1e3, lamWorst - 0.42, sprintf('worst  %.0f mm', lY(iw)*1e3), ...
     'Color', BLUE, 'FontSize', 8.5, 'HorizontalAlignment','center');
plot(P.l*1e3, me.lambda, 's', 'MarkerSize', 10, 'MarkerFaceColor', INK, ...
     'MarkerEdgeColor','w','LineWidth',1.2);
text(P.l*1e3 + 3, me.lambda + 0.42, sprintf('l = %.1f mm\n\\lambda_3 = %.2f', ...
     P.l*1e3, me.lambda), 'FontWeight','bold','FontSize',9);
xlabel('CoG height above the axle  l  [mm]'); ylabel('\lambda_3  [1/s]');
title(sprintf('Unstable pole vs CoG height (moving %.0f g)', P.mMove*1e3));
ylim([0.95*min(lamY) 1.09*max(lamY)]);

% --- (3) s-plane pole map with parameter loci ------------------------------
subplot(2,2,3); hold on; grid on; box off;
pR = zeros(numel(rr),3);  pY = zeros(numel(yy),3);
for k = 1:numel(rr), pR(k,:) = sort(model(P, rr(k), P.l, P.Jc).poles,'descend')'; end
for k = 1:numel(yy)
    [lk, Jck] = moveMass(P, yy(k));
    pY(k,:) = sort(model(P, P.r, lk, Jck).poles,'descend')';
end
plot(pR(:,1), 0.10*ones(numel(rr),1), '-',  'Color', BLUE,   'LineWidth', 3);
plot(pR(:,3), 0.10*ones(numel(rr),1), '-',  'Color', BLUE,   'LineWidth', 3);
plot(pY(:,1),-0.10*ones(numel(yy),1), '-',  'Color', ORANGE, 'LineWidth', 3);
plot(pY(:,3),-0.10*ones(numel(yy),1), '-',  'Color', ORANGE, 'LineWidth', 3);
% filled marker = small parameter, open marker = large parameter
plot([pR(1,1) pR(1,3)],   [0.10 0.10], 'o','Color',BLUE,  'MarkerFaceColor',BLUE,  'MarkerSize',5);
plot([pR(end,1) pR(end,3)],[0.10 0.10],'o','Color',BLUE,  'MarkerFaceColor','w',   'MarkerSize',5);
plot([pY(1,1) pY(1,3)],   [-0.10 -0.10],'o','Color',ORANGE,'MarkerFaceColor',ORANGE,'MarkerSize',5);
plot([pY(end,1) pY(end,3)],[-0.10 -0.10],'o','Color',ORANGE,'MarkerFaceColor','w',  'MarkerSize',5);
plot([0 0], [-1 1], '-', 'Color', [0.8 0.8 0.8], 'LineWidth', 1);
plot(real(me.poles), zeros(3,1), 'x', 'Color', INK, 'MarkerSize', 13, 'LineWidth', 2.2);
text(me.lambda, 0.30, sprintf(' +%.2f', me.lambda), 'FontWeight','bold','FontSize',9);
text(-me.lambda,0.30, sprintf('-%.2f ', me.lambda), 'FontWeight','bold','FontSize',9, ...
     'HorizontalAlignment','right');
text(0, 0.30, ' 0 (wheel position)', 'FontSize', 8.5, 'Color', GREY);
text(max(pR(:,1))*0.98, 0.22, sprintf('r:  %.0f \\rightarrow %.0f mm', ...
     S.rRange(1)*1e3, S.rRange(2)*1e3), 'Color', BLUE, 'FontSize', 8.5, ...
     'HorizontalAlignment','right');
text(max(pY(:,1))*0.98,-0.26, sprintf('battery:  %.0f \\rightarrow %.0f mm', ...
     S.yRange(1)*1e3, S.yRange(2)*1e3), 'Color', ORANGE, ...
     'FontSize', 8.5, 'HorizontalAlignment','right');
xlabel('Re\{s\}  [1/s]'); ylabel('Im\{s\}  [1/s]');
title('Open-loop poles (all real - no open-loop oscillation)');
ylim([-1 1]); xlim(1.15*max(abs(pR(:,1)))*[-1 1]);

% --- (4) lambda_3 over the whole (r, battery height) design space ----------
subplot(2,2,4); hold on; box off;
rg = linspace(S.rRange(1), S.rRange(2), 90);
yg = linspace(S.yRange(1), S.yRange(2), 90);
L  = zeros(numel(yg), numel(rg));
for i = 1:numel(yg)
    [li, Jci] = moveMass(P, yg(i));
    for j = 1:numel(rg)
        L(i,j) = model(P, rg(j), li, Jci).lambda;
    end
end
contourf(rg*1e3, yg*1e3, L, 18, 'LineColor','none');
[C,h] = contour(rg*1e3, yg*1e3, L, (round(me.lambda*10)/10)*[1 1], 'LineColor','w','LineWidth',2);
clabel(C, h, 'Color','w','FontWeight','bold','FontSize',8);
try, cmap = parula(256); catch, cmap = jet(256); end    % parula is MATLAB-only
colormap(flipud(cmap)); cb = colorbar; ylabel(cb, '\lambda_3  [1/s]');
plot(P.r*1e3, P.yMove*1e3, 's', 'MarkerSize', 11, 'MarkerFaceColor','k', ...
     'MarkerEdgeColor','w','LineWidth',1.5);
text(P.r*1e3 + 4, P.yMove*1e3, ' your build', 'Color','w','FontWeight','bold','FontSize',9);
xlabel('wheel radius  r  [mm]'); ylabel('battery height above the axle  [mm]');
title('Design space (lighter = slower fall = easier)');

%% ============================== FIGURE 2 ===================================
figure('Name','Root locus','Color','w','Position',[120 120 1000 440]);

% --- (1) lean-only feedback: omega = K*theta -------------------------------
subplot(1,2,1); hold on; grid on; box off;
KK = linspace(-S.Kmax, S.Kmax, 600);
a  = me.A0*P.b_theta + me.B0*KK;                 % coefficient of s
s1 = (-a + sqrt(a.^2 + 4*me.lambda^2))/2;
s2 = (-a - sqrt(a.^2 + 4*me.lambda^2))/2;
plot(s1, zeros(size(KK)), '.', 'Color', ORANGE, 'MarkerSize', 6);
plot(s2, zeros(size(KK)), '.', 'Color', BLUE,   'MarkerSize', 6);
plot([0 0], [-1 1], '-', 'Color', [0.8 0.8 0.8], 'LineWidth', 1);
plot(real(me.poles), zeros(3,1), 'x', 'Color', INK, 'MarkerSize', 13, 'LineWidth', 2.2);
plot(0, 0, 'o', 'MarkerSize', 10, 'MarkerEdgeColor', INK, 'MarkerFaceColor','none', ...
     'LineWidth', 1.6);
text(me.lambda*0.35, 0.62, sprintf(['this branch never crosses into\n', ...
     'the left half plane, for any K']), 'Color', ORANGE, 'FontSize', 8.5);
xlabel('Re\{s\}  [1/s]'); ylabel('Im\{s\}  [1/s]');
title('Lean feedback alone:  \omega = K\theta');
ylim([-1 1]); xlim(1.3*max(abs([s1 s2]))*[-1 1]);

% --- (2) what full-state feedback does -------------------------------------
subplot(1,2,2); hold on; grid on; box off;
plot([0 0], [-6 6], '-', 'Color', [0.8 0.8 0.8], 'LineWidth', 1);
hOL = plot(real(me.poles), zeros(3,1), 'x', 'Color', INK, 'MarkerSize', 13, 'LineWidth', 2.2);
hCL = [];  pcl = [];  txt = 'LQR needs the Control System Toolbox';
if exist('lqr','file')
    Q = diag([1 100 10]);  Rw = 1;
    K = lqr(me.A, me.B, Q, Rw);
    pcl = eig(me.A - me.B*K);
    hCL = plot(real(pcl), imag(pcl), 'o', 'MarkerSize', 9, 'MarkerFaceColor',[0.11 0.69 0.48], ...
         'MarkerEdgeColor','w','LineWidth',1.2);
    txt = sprintf('LQR  Q = diag(%g,%g,%g),  R = %g\nK = [%.2f  %.2f  %.2f]', ...
                  diag(Q), Rw, K);
    fprintf('  LQR       K = [%.4f  %.4f  %.4f]\n', K);
    fprintf('            closed-loop poles: %s\n', ...
            strtrim(sprintf('%+.3f ', sort(real(pcl),'descend'))));
end
text(0.03, 0.06, txt, 'Units','normalized','FontSize', 8.5, 'Color', GREY);
if isempty(hCL)
    legend(hOL, {'open loop'}, 'Location','northwest','Box','off');
else
    legend([hOL(1) hCL(1)], {'open loop','closed loop'}, 'Location','northwest','Box','off');
end
xlabel('Re\{s\}  [1/s]'); ylabel('Im\{s\}  [1/s]');
title('Full-state feedback:  \omega = -K[\phi;\theta;p_\theta]');
ax = 1.25*max([me.lambda; abs(pcl(:))]);
xlim(ax*[-1 1]); ylim(ax*[-1 1]);


%% ============================ local functions ==============================
function P = derive(P)
    P.mw = 2*P.mWheel;
    P.Jr = 2*P.Jrotor;
    if strcmpi(P.JcMode,'rig')
        % rig pivots on the axle, wheels off:
        %   lambdaRig^2 = mc*g*l / (Jc + mc*l^2 + Jr)
        P.Jc = P.mc*P.g*P.l/P.lambdaRig^2 - P.mc*P.l^2 - P.Jr;
    else
        P.Jc = P.JcDirect;
    end
    P.JaxleRef = P.Jc + P.mc*P.l^2;     % chassis inertia about the axle
end

function m = model(P, r, l, Jc)
    m.Jw = P.mw*0.5*r^2 + P.Jr;                       % 2 x (1/2 m r^2) + 2 x Jrotor
    m.D  = m.Jw + Jc + P.mw*r^2 + P.mc*(r + l)^2;     % = M22
    m.A0 = 1/m.D;
    m.B0 = (m.Jw + P.mw*r^2 + P.mc*r*(r + l))*m.A0;
    m.A  = [0  0            0
            0  0            m.A0
            0  P.mc*P.g*l  -P.b_theta*m.A0];
    m.B  = [1; -m.B0; P.b_theta*m.B0];
    m.poles  = eig(m.A);
    m.lambda = sqrt(m.A0*P.mc*P.g*l);                 % = max pole when b_theta = 0
end

function [l, Jc] = moveMass(P, y)
    % Move mMove from its present height yMove to y. l and Jc move together.
    if P.mMove <= 0
        l = P.l;  Jc = P.Jc;  return
    end
    l     = P.l + P.mMove*(y - P.yMove)/P.mc;
    Jaxle = P.JaxleRef + P.mMove*(y^2 - P.yMove^2);
    Jc    = Jaxle - P.mc*l^2;
end
