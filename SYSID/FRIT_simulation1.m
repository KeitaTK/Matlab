%% メインスクリプト：シミュレーション実行
clear;
%% 関数ハンドルの読み込み
mat_dir = fullfile(fileparts(mfilename('fullpath')), 'mat_data');
load(fullfile(mat_dir, 'dX.mat'),    'fdX');
load(fullfile(mat_dir, 'R_BtoI.mat'),'fR_BtoI');
load(fullfile(mat_dir, 'Omega.mat'), 'fOmega');
load(fullfile(mat_dir, 'P_BtoI.mat'),'fP_BtoI');
load(fullfile(mat_dir, 'P_ItoB.mat'),'fP_ItoB');

%% 初期値設定
X0      = [0; 0; 0];                    % 位置
dX0     = [0; 0; 0];                    % 速度
eta0    = [0; 0; 0] * pi/180;        % ロール・ピッチ・ヨー
deta0   = [0; 0; 0];                    % 角速度
Current0 = [X0; eta0; dX0; deta0];

%% 制御器設定
etad       = [pi/12; 0; 0];                 % 目標角度
kp         = [100; 16; 50];               % 比例ゲイン
kd         = [18; 6; 5];                 % 微分ゲイン
setting_val = [etad; kp; kd];

%% シミュレーション条件
tspan   = 0 : 0.01 : 10;               % 時間
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

%% 目標モデル定義（プロット用）
tau_d = 0.6;
Mc = tf(1, [tau_d 1]);

%% 結果プロット
figure(1);
subplot(3,1,1); plot(ta, Current(:,4), 'b'); hold on;
step(Mc, ta, 'k--');
title('ロール角 φ'); xlabel('時刻 [s]'); ylabel('rad'); legend('シミュレーション','目標モデル');
subplot(3,1,2); plot(ta, Current(:,5), 'b'); hold on;
step(Mc, ta, 'k--');
title('ピッチ角 θ'); xlabel('時刻 [s]'); ylabel('rad'); legend('シミュレーション','目標モデル');
subplot(3,1,3); plot(ta, Current(:,6), 'b'); hold on;
step(Mc, ta, 'k--');
title('ヨー角 Ψ'); xlabel('時刻 [s]'); ylabel('rad'); legend('シミュレーション','目標モデル');

figure(2);
subplot(3,1,1), plot(ta, Current(:,10)), title('dφ'), xlabel('s'), ylabel('rad/s');
subplot(3,1,2), plot(ta, Current(:,11)), title('dθ'), xlabel('s'), ylabel('rad/s');
subplot(3,1,3), plot(ta, Current(:,12)), title('dΨ'), xlabel('s'), ylabel('rad/s');

ta_init     = ta;
angle_init  = Current(:,4:6);    % φ θ Ψ
rate_init   = Current(:,10:12);  % dφ dθ dΨ

%% CSV出力
csv_table = table();
csv_table.t      = ta;                  % 時刻
csv_table.phi    = Current(:,4);        % ロール角（慣性系）
csv_table.theta  = Current(:,5);        % ピッチ角（慣性系）
csv_table.psi    = Current(:,6);        % ヨー角（慣性系)

% 機体座標系角度・トルク指令を計算して格納
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

    % 機体座標系角度を保存
    phi_b(i)   = Eta_b(1);
    theta_b(i) = Eta_b(2);
    psi_b(i)   = Eta_b(3);

    % トルク指令（機体座標系, [N*m]）
    u = -kp .* (Eta_b - etad) - kd .* dEta_b;
    tau_cmd(i,:) = u(:).';
end

% CSV列を追加
csv_table.phi_b   = phi_b;
csv_table.theta_b = theta_b;
csv_table.psi_b   = psi_b;

csv_table.tau_x = tau_cmd(:,1);
csv_table.tau_y = tau_cmd(:,2);
csv_table.tau_z = tau_cmd(:,3);

writetable(csv_table, 'current_FRIT.csv');

clearvars -except ta_init angle_init rate_init

% CSV読み込み
T = readtable('current_FRIT.csv');

% 出力 y: 姿勢
y = [T.phi_b, T.theta_b, T.psi_b]; 

% 入力 u: 3軸トルク指令 tau_cmd
u = [T.tau_x, T.tau_y, T.tau_z];    

% 時刻
t = T.t;
Ts = median(diff(t));

%% 1) 参照モデルの設定
tau_d = 0.6;
Mc = tf(1, [tau_d 1]);         
M  = c2d(Mc, Ts, 'tustin');

%% 2) 仮想参照 r_tilde と誤差 e_tilde の生成
Minv = minreal(1/M);   % SISO
N = size(y,1); nAx = size(y,2);
r_tilde = zeros(N, nAx);
for i = 1:nAx
    r_tilde(:,i) = lsim(Minv, y(:,i), t);  
end
e_tilde = r_tilde - y;

%% 3) 回帰と推定
alpha  = 0.1;    
lambda = 1e-4;   
k0 = 2;          

kp = zeros(1,nAx); kd = zeros(1,nAx);
rmse = zeros(1,nAx); vaf = zeros(1,nAx);

for i = 1:nAx
    e = e_tilde(:,i);

    de = [0; diff(e)]/Ts;
    D  = filter((1-alpha), [1 -alpha], de);

    ei = e(k0:end);
    Di = D(k0:end);
    u_fit = u(k0:end, i);

    % 回帰行列
    Phi = [ei, Di];

    % パラメータ推定
    theta = (Phi.'*Phi + lambda*eye(2)) \ (Phi.'*u_fit);
    kp(i) = theta(1); kd(i) = theta(2);

end

%% 4) 結果表示
names = {'roll','pitch','yaw'};
for ax = 1:nAx
    fprintf('%s: kp=%.6f [N*m/rad], kd=%.6f [N*m/(rad/s)]\n', names{ax}, kp(ax), kd(ax));
end
mat_dir = fullfile(fileparts(mfilename('fullpath')), 'mat_data');
if ~exist(mat_dir, 'dir')
    mkdir(mat_dir);
end
save(fullfile(mat_dir, 'FRIT_roll_angle1_PD.mat'), 'kp', 'kd', 'tau_d', 'Ts');

%% FRITシミュレーション実行
clearvars -except ta_init angle_init rate_init
%% 関数ハンドルの読み込み
mat_dir = fullfile(fileparts(mfilename('fullpath')), 'mat_data');
load(fullfile(mat_dir, "dX.mat"),    "fdX");
load(fullfile(mat_dir, "R_BtoI.mat"),"fR_BtoI");
load(fullfile(mat_dir, "Omega.mat"), "fOmega");
load(fullfile(mat_dir, "P_BtoI.mat"),"fP_BtoI");
load(fullfile(mat_dir, "P_ItoB.mat"),"fP_ItoB");

%% 初期値設定
X0      = [0; 0; 0];                    % 位置
dX0     = [0; 0; 0];                    % 速度
eta0    = [0; 0; 0] * pi/180;        % ロール・ピッチ・ヨー
deta0   = [0; 0; 0];                    % 角速度
Current0 = [X0; eta0; dX0; deta0];

%% 制御器設定
etad = [pi/12; 0; 0];  % 目標角度（機体座標系）

% FRIT推定結果の読み込み
frit = load(fullfile(mat_dir, 'FRIT_roll_angle1_PD.mat'), 'kp', 'kd');
kp = frit.kp(:);   
kd = frit.kd(:);   
% --- 不要な表示は削除 ---

setting_val = [etad; kp; kd];

%% シミュレーション条件
tspan   = 0 : 0.01 : 10;               % 時間
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

%% 結果プロット
figure(3);
subplot(3,1,1), plot(ta, Current(:,4)), title('ロール角 φ'),  xlabel('時刻 [s]'), ylabel('rad');
subplot(3,1,2), plot(ta, Current(:,5)), title('ピッチ角 θ'), xlabel('時刻 [s]'), ylabel('rad');
subplot(3,1,3), plot(ta, Current(:,6)), title('ヨー角 Ψ'),  xlabel('時刻 [s]'), ylabel('rad');

figure(4);
subplot(3,1,1), plot(ta, Current(:,10)), title('dφ'), xlabel('s'), ylabel('rad/s');
subplot(3,1,2), plot(ta, Current(:,11)), title('dθ'), xlabel('s'), ylabel('rad/s');
subplot(3,1,3), plot(ta, Current(:,12)), title('dΨ'), xlabel('s'), ylabel('rad/s');

figure(5);
titles_angle = {'ロール角 φ','ピッチ角 θ','ヨー角 Ψ'};
for i = 1:3
    subplot(3,1,i); hold on;
    plot(ta_init, angle_init(:,i), 'b-', 'LineWidth',1.2);
    plot(ta,       Current(:,3+i), 'r--', 'LineWidth',1.2); % 4,5,6列
    xlabel('時刻 [s]'); ylabel('rad'); title([titles_angle{i},]);
    legend('初期ゲイン','FRITゲイン','Location','best'); grid on;
end

figure(6);
titles_rate = {'dφ','dθ','dΨ'};
for i = 1:3
    subplot(3,1,i); hold on;
    plot(ta_init, rate_init(:,i), 'b-', 'LineWidth',1.2);
    plot(ta,       Current(:,9+i), 'r--', 'LineWidth',1.2); % 10,11,12列
    xlabel('時刻 [s]'); ylabel('rad/s'); title([titles_rate{i},]);
    legend('初期ゲイン','FRITゲイン','Location','best'); grid on;
end

%% CSV出力
csv_table = table();
csv_table.t      = ta;                  % 時刻
csv_table.phi    = Current(:,4);        % ロール角（慣性系）
csv_table.theta  = Current(:,5);        % ピッチ角（慣性系）
csv_table.psi    = Current(:,6);        % ヨー角（慣性系)

% 機体座標系角度・トルク指令を計算して格納
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

    % 機体座標系角度を保存
    phi_b(i)   = Eta_b(1);
    theta_b(i) = Eta_b(2);
    psi_b(i)   = Eta_b(3);

    % トルク指令（機体座標系, [N*m]）
    u = -kp .* (Eta_b - etad) - kd .* dEta_b;
    tau_cmd(i,:) = u(:).';
end

% CSV列を追加
csv_table.phi_b   = phi_b;
csv_table.theta_b = theta_b;
csv_table.psi_b   = psi_b;

csv_table.tau_x = tau_cmd(:,1);
csv_table.tau_y = tau_cmd(:,2);
csv_table.tau_z = tau_cmd(:,3);

writetable(csv_table, 'current_FRIT.csv');

%% system_dynamics 関数
function dCurrent = system_dynamics(~, Current, const, setting_val, fdX, fR_BtoI, fOmega, fP_BtoI, fP_ItoB)
    % 状態ベクトル分解
    X_I    = Current(1:3);
    Eta_I  = Current(4:6);
    dX_I   = Current(7:9);
    dEta_I = Current(10:12);

    % 制御目標・ゲイン
    etad = setting_val(1:3);
    kp   = setting_val(4:6);
    kd   = setting_val(7:9);

    % 変換行列・角速度を計算
    R_BtoI = fR_BtoI(Eta_I(1), Eta_I(2), Eta_I(3));
    Omega  = fOmega( ...
        Eta_I(1), Eta_I(2), Eta_I(3), ...
        dEta_I(1), dEta_I(2), dEta_I(3) );
    P_BtoI = fP_BtoI(Eta_I(1), Eta_I(2), Eta_I(3));
    P_ItoB = fP_ItoB(Eta_I(1), Eta_I(2), Eta_I(3));

    % 機体座標系誤差
    Eta_b  = P_ItoB * Eta_I;
    dEta_b = P_ItoB * dEta_I;

    % PID 制御入力
    u        = -kp .* (Eta_b - etad) - kd .* dEta_b;
    input_sim = u * 100/(1023-551);

    % モーター力・トルク
    motor_torq = 4 * diag([0.1425,0.090,1]) * ...
                 diag([0.02219,0.02219,0.0003533]) * input_sim;
    F_b        = [0;0;1] * 80 * 0.02219 * 4;
    Tau_b      = motor_torq;

    % 外力・モーメントを慣性系へ
    F_I       = R_BtoI * F_b;
    Tau_I     = P_BtoI * Tau_b;
    Q_I       = [F_I; Tau_I];

    % fdX への引数リストを cell 配列に変換
    arg_list  = num2cell([ ...
        X_I.', ...
        Eta_I.', ...
        dX_I.', ...
        dEta_I.', ...
        Q_I.', ...
        const(:).' ...
    ]);

    % 状態微分計算
    dCurrent = fdX(arg_list{:});
end
