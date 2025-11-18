%% メインスクリプト：シミュレーション実行
clear;
%% 関数ハンドルの読み込み
mat_dir = fullfile(pwd, 'mat_data');
load(fullfile(mat_dir, "dX.mat"),    "fdX");
load(fullfile(mat_dir, "R_BtoI.mat"),"fR_BtoI");
load(fullfile(mat_dir, "Omega.mat"), "fOmega");
load(fullfile(mat_dir, "P_BtoI.mat"),"fP_BtoI");
load(fullfile(mat_dir, "P_ItoB.mat"),"fP_ItoB");

%% 初期値設定
X0      = [0; 0; 0];                    % 位置
dX0     = [0; 0; 0];                    % 速度
eta0    = [40; 20; 10] * pi/180;        % ロール・ピッチ・ヨー
deta0   = [0; 0; 0];                    % 角速度
Current0 = [X0; eta0; dX0; deta0];

%% 制御器設定
etad       = [0; 0; 0];                 % 目標角度
kp         = [5; 16; 50];               % 比例ゲイン
kd         = [6; 6; 5];                 % 微分ゲイン
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
figure(1);
subplot(3,1,1), plot(ta, Current(:,4)), title('ロール角 φ'),  xlabel('時刻 [s]'), ylabel('rad');
subplot(3,1,2), plot(ta, Current(:,5)), title('ピッチ角 θ'), xlabel('時刻 [s]'), ylabel('rad');
subplot(3,1,3), plot(ta, Current(:,6)), title('ヨー角 Ψ'),  xlabel('時刻 [s]'), ylabel('rad');

figure(2);
subplot(3,1,1), plot(ta, Current(:,10)), title('dφ'), xlabel('s'), ylabel('rad/s');
subplot(3,1,2), plot(ta, Current(:,11)), title('dθ'), xlabel('s'), ylabel('rad/s');
subplot(3,1,3), plot(ta, Current(:,12)), title('dΨ'), xlabel('s'), ylabel('rad/s');

figure(3);
subplot(3,1,1), plot(ta, Current(:,1)), title('位置 x'), xlabel('s'), ylabel('m');
subplot(3,1,2), plot(ta, Current(:,2)), title('位置 y'), xlabel('s'), ylabel('m');
subplot(3,1,3), plot(ta, Current(:,3)), title('位置 z'), xlabel('s'), ylabel('m');

figure(4);
subplot(3,1,1), plot(ta, Current(:,7)), title('速度 dx'), xlabel('s'), ylabel('m/s');
subplot(3,1,2), plot(ta, Current(:,8)), title('速度 dy'), xlabel('s'), ylabel('m/s');
subplot(3,1,3), plot(ta, Current(:,9)), title('速度 dz'), xlabel('s'), ylabel('m/s');


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
