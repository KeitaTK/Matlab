function [kp, kd] = estimate_vrft_gains(kp_init, kd_init)
% estimate_vrft_gains - VRFT法によるゲイン推定
%
% 入力:
%   kp_init: 初期比例ゲイン [3x1]
%   kd_init: 初期微分ゲイン [3x1]
%
% 出力:
%   kp: 推定比例ゲイン [3x1]
%   kd: 推定微分ゲイン [3x1]

%% 関数ハンドルの読み込み
mat_dir = fullfile(fileparts(mfilename('fullpath')), 'mat_data');
load(fullfile(mat_dir, 'dX.mat'),    'fdX');
load(fullfile(mat_dir, 'R_BtoI.mat'),'fR_BtoI');
load(fullfile(mat_dir, 'Omega.mat'), 'fOmega');
load(fullfile(mat_dir, 'P_BtoI.mat'),'fP_BtoI');
load(fullfile(mat_dir, 'P_ItoB.mat'),'fP_ItoB');

%% 初期値設定
X0      = [0; 0; 0];
dX0     = [0; 0; 0];
eta0    = [0; 0; 0] * pi/180;
deta0   = [0; 0; 0];
Current0 = [X0; eta0; dX0; deta0];

%% 制御器設定
etad       = [pi/12; 0; 0];
kp_vec     = kp_init(:); % 3x1に変換
kd_vec     = kd_init(:);
setting_val = [etad; kp_vec; kd_vec];

%% シミュレーション条件
tspan   = 0 : 0.01 : 10;
options = odeset('RelTol',1e-6,'AbsTol',1e-6);

%% 定数設定
J_xx = 0.01; J_yy = 0.01; J_zz = 0.01; 
m    = 0.7;  g    = 9.81;
mu_1 = 0.1;  mu_2 = 0.01;
const = [J_xx; J_yy; J_zz; m; g; mu_1; mu_2];

%% ODE シミュレーション
[ta, Current] = ode45( ...
    @(t, y) system_dynamics(t, y, const, setting_val, fdX, fR_BtoI, fOmega, fP_BtoI, fP_ItoB), ...
    tspan, Current0, options);

%% CSV出力
csv_table = table();
csv_table.t      = ta;
csv_table.phi    = Current(:,4);
csv_table.theta  = Current(:,5);
csv_table.psi    = Current(:,6);

N = length(ta);
phi_b   = zeros(N,1);
theta_b = zeros(N,1);
psi_b   = zeros(N,1);
tau_cmd = zeros(N,3);

for i = 1:N
    Eta_I  = Current(i,4:6).';
    dEta_I = Current(i,10:12).';
    P_ItoB = fP_ItoB(Eta_I(1), Eta_I(2), Eta_I(3));

    Eta_b  = P_ItoB * Eta_I;
    dEta_b = P_ItoB * dEta_I;

    phi_b(i)   = Eta_b(1);
    theta_b(i) = Eta_b(2);
    psi_b(i)   = Eta_b(3);

    u = -kp_vec .* (Eta_b - etad) - kd_vec .* dEta_b;
    tau_cmd(i,:) = u(:).';
end

csv_table.phi_b   = phi_b;
csv_table.theta_b = theta_b;
csv_table.psi_b   = psi_b;
csv_table.tau_x = tau_cmd(:,1);
csv_table.tau_y = tau_cmd(:,2);
csv_table.tau_z = tau_cmd(:,3);

%% VRFT推定処理
t = csv_table.t; 
Ts = median(diff(t));
y = [csv_table.phi_b, csv_table.theta_b, csv_table.psi_b];
u = [csv_table.tau_x, csv_table.tau_y, csv_table.tau_z];

% 目標閉ループモデル
tau_d   = 0.6;
alpha_m = exp(-Ts/tau_d);

% 仮想参照と誤差
rv = zeros(size(y));
ev = zeros(size(y));
for k = 1:3
    rv(:,k) = filter([1, -alpha_m], 1 - alpha_m, y(:,k));  
    ev(:,k) = rv(:,k) - y(:,k);
end

% 誤差微分（LPF付き）
tau_f   = 0.05;
alpha_f = exp(-Ts/tau_f);
ed_raw  = [zeros(1,3); diff(ev)] / Ts;
ed      = filter(1 - alpha_f, [1, -alpha_f], ed_raw);

% PD推定
kp = zeros(3,1); kd = zeros(3,1);
tol = 1e-9;
for ax = 1:3
    Phi = [ev(:,ax), ed(:,ax)];
    if any(vecnorm(Phi) > tol)
        theta = Phi \ u(:,ax);
        kp(ax) = theta(1);
        kd(ax) = theta(2);
    end
end

end

%% system_dynamics 関数
function dCurrent = system_dynamics(~, Current, const, setting_val, fdX, fR_BtoI, fOmega, fP_BtoI, fP_ItoB)
    X_I    = Current(1:3);
    Eta_I  = Current(4:6);
    dX_I   = Current(7:9);
    dEta_I = Current(10:12);

    etad = setting_val(1:3);
    kp   = setting_val(4:6);
    kd   = setting_val(7:9);

    R_BtoI = fR_BtoI(Eta_I(1), Eta_I(2), Eta_I(3));
    P_BtoI = fP_BtoI(Eta_I(1), Eta_I(2), Eta_I(3));
    P_ItoB = fP_ItoB(Eta_I(1), Eta_I(2), Eta_I(3));

    Eta_b  = P_ItoB * Eta_I;
    dEta_b = P_ItoB * dEta_I;

    u        = -kp .* (Eta_b - etad) - kd .* dEta_b;
    input_sim = u * 100/(1023-551);

    motor_torq = 4 * diag([0.1425,0.090,1]) * ...
                 diag([0.02219,0.02219,0.0003533]) * input_sim;
    F_b        = [0;0;1] * 80 * 0.02219 * 4;
    Tau_b      = motor_torq;

    F_I       = R_BtoI * F_b;
    Tau_I     = P_BtoI * Tau_b;
    Q_I       = [F_I; Tau_I];

    arg_list  = num2cell([ ...
        X_I.', ...
        Eta_I.', ...
        dX_I.', ...
        dEta_I.', ...
        Q_I.', ...
        const(:).' ...
    ]);

    dCurrent = fdX(arg_list{:});
end
