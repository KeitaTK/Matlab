function [kp, kd] = estimate_frit_gains(kp_init, kd_init)
% estimate_frit_gains - FRIT法によるゲイン推定
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
kp_vec     = kp_init(:);
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

%% FRIT推定処理
y = [csv_table.phi_b, csv_table.theta_b, csv_table.psi_b]; 
u = [csv_table.tau_x, csv_table.tau_y, csv_table.tau_z];    
t = csv_table.t;
Ts = median(diff(t));

% 参照モデル
tau_d = 0.6;
Mc = tf(1, [tau_d 1]);         
M  = c2d(Mc, Ts, 'tustin');   

% 仮想参照と誤差
Minv = minreal(1/M);
N = size(y,1); nAx = size(y,2);
r_tilde = zeros(N, nAx);
for i = 1:nAx
    r_tilde(:,i) = lsim(Minv, y(:,i), t);  
end
e_tilde = r_tilde - y;

% 回帰と推定
alpha  = 0.1;    
lambda = 1e-4;   
k0 = 2;          

kp = zeros(1,nAx); kd = zeros(1,nAx);

for i = 1:nAx
    e = e_tilde(:,i);
    de = [0; diff(e)]/Ts;
    D  = filter((1-alpha), [1 -alpha], de);

    ei = e(k0:end);
    Di = D(k0:end);
    u_fit = u(k0:end, i);

    Phi = [ei, Di];
    theta = (Phi.'*Phi + lambda*eye(2)) \ (Phi.'*u_fit);
    kp(i) = theta(1); kd(i) = theta(2);
end

kp = kp(:); % 3x1に変換
kd = kd(:);

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
