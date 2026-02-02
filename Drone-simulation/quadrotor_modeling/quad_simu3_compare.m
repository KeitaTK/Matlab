%% クアッドコプター姿勢制御のパラメータ比較シミュレーション
% 異なるPD制御ゲインでの応答性能を比較
clear; close all;

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
eta0    = [40; 20; 10] * pi/180;        % ロール・ピッチ・ヨー（初期角度）
deta0   = [0; 0; 0];                    % 角速度
Current0 = [X0; eta0; dX0; deta0];

%% 制御器設定（3ケース）
etad = [0; 0; 0];  % 目標角度（すべて0度）

% Case 1: 低ゲイン
kp1 = [2.5; 8; 25];
kd1 = [3; 3; 2.5];

% Case 2: 基準ゲイン（元の設定）
kp2 = [5; 16; 50];
kd2 = [6; 6; 5];

% Case 3: 高ゲイン
kp3 = [10; 32; 100];
kd3 = [12; 12; 10];

%% シミュレーション条件
tspan   = 0 : 0.01 : 5;  % 5秒間のシミュレーション
options = odeset('RelTol',1e-6,'AbsTol',1e-6);

%% 定数設定
J_xx = 0.01; J_yy = 0.01; J_zz = 0.01; 
m    = 0.7;  g    = 9.81;
mu_1 = 0.1;  mu_2 = 0.01;
const = [J_xx; J_yy; J_zz; m; g; mu_1; mu_2];

%% Case 1: 低ゲイン
fprintf('Case 1: 低ゲイン実行中...\n');
setting_val1 = [etad; kp1; kd1];
[ta1, Current1] = ode45( ...
    @(t, y) system_dynamics(t, y, const, setting_val1, fdX, fR_BtoI, fOmega, fP_BtoI, fP_ItoB), ...
    tspan, Current0, options);

%% Case 2: 基準ゲイン
fprintf('Case 2: 基準ゲイン実行中...\n');
setting_val2 = [etad; kp2; kd2];
[ta2, Current2] = ode45( ...
    @(t, y) system_dynamics(t, y, const, setting_val2, fdX, fR_BtoI, fOmega, fP_BtoI, fP_ItoB), ...
    tspan, Current0, options);

%% Case 3: 高ゲイン
fprintf('Case 3: 高ゲイン実行中...\n');
setting_val3 = [etad; kp3; kd3];
[ta3, Current3] = ode45( ...
    @(t, y) system_dynamics(t, y, const, setting_val3, fdX, fR_BtoI, fOmega, fP_BtoI, fP_ItoB), ...
    tspan, Current0, options);

%% 結果プロット: ロール角の比較
figure(1);
set(gcf, 'Position', [100, 100, 1200, 400]);
subplot(1,3,1)
plot(ta1, Current1(:,4)*180/pi, 'LineWidth', 2); hold on;
plot(ta2, Current2(:,4)*180/pi, 'LineWidth', 2);
plot(ta3, Current3(:,4)*180/pi, 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1);
xlabel('時刻 [s]', 'FontSize', 12);
ylabel('ロール角 φ [deg]', 'FontSize', 12);
title('ロール角応答の比較', 'FontSize', 14);
legend('低ゲイン', '基準ゲイン', '高ゲイン', '目標値', 'Location', 'best');
grid on;

subplot(1,3,2)
plot(ta1, Current1(:,5)*180/pi, 'LineWidth', 2); hold on;
plot(ta2, Current2(:,5)*180/pi, 'LineWidth', 2);
plot(ta3, Current3(:,5)*180/pi, 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1);
xlabel('時刻 [s]', 'FontSize', 12);
ylabel('ピッチ角 θ [deg]', 'FontSize', 12);
title('ピッチ角応答の比較', 'FontSize', 14);
legend('低ゲイン', '基準ゲイン', '高ゲイン', '目標値', 'Location', 'best');
grid on;

subplot(1,3,3)
plot(ta1, Current1(:,6)*180/pi, 'LineWidth', 2); hold on;
plot(ta2, Current2(:,6)*180/pi, 'LineWidth', 2);
plot(ta3, Current3(:,6)*180/pi, 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1);
xlabel('時刻 [s]', 'FontSize', 12);
ylabel('ヨー角 ψ [deg]', 'FontSize', 12);
title('ヨー角応答の比較', 'FontSize', 14);
legend('低ゲイン', '基準ゲイン', '高ゲイン', '目標値', 'Location', 'best');
grid on;

%% 角速度の比較
figure(2);
set(gcf, 'Position', [100, 100, 1200, 400]);
subplot(1,3,1)
plot(ta1, Current1(:,10)*180/pi, 'LineWidth', 2); hold on;
plot(ta2, Current2(:,10)*180/pi, 'LineWidth', 2);
plot(ta3, Current3(:,10)*180/pi, 'LineWidth', 2);
xlabel('時刻 [s]', 'FontSize', 12);
ylabel('角速度 dφ/dt [deg/s]', 'FontSize', 12);
title('ロール角速度の比較', 'FontSize', 14);
legend('低ゲイン', '基準ゲイン', '高ゲイン', 'Location', 'best');
grid on;

subplot(1,3,2)
plot(ta1, Current1(:,11)*180/pi, 'LineWidth', 2); hold on;
plot(ta2, Current2(:,11)*180/pi, 'LineWidth', 2);
plot(ta3, Current3(:,11)*180/pi, 'LineWidth', 2);
xlabel('時刻 [s]', 'FontSize', 12);
ylabel('角速度 dθ/dt [deg/s]', 'FontSize', 12);
title('ピッチ角速度の比較', 'FontSize', 14);
legend('低ゲイン', '基準ゲイン', '高ゲイン', 'Location', 'best');
grid on;

subplot(1,3,3)
plot(ta1, Current1(:,12)*180/pi, 'LineWidth', 2); hold on;
plot(ta2, Current2(:,12)*180/pi, 'LineWidth', 2);
plot(ta3, Current3(:,12)*180/pi, 'LineWidth', 2);
xlabel('時刻 [s]', 'FontSize', 12);
ylabel('角速度 dψ/dt [deg/s]', 'FontSize', 12);
title('ヨー角速度の比較', 'FontSize', 14);
legend('低ゲイン', '基準ゲイン', '高ゲイン', 'Location', 'best');
grid on;

%% 性能指標の計算
% 整定時間（2%基準）
settling_threshold = 0.02; % 2%
fprintf('\n=== 性能評価 ===\n');

% ロール角（Case2）
final_val = 0; % 目標値
initial_val = eta0(1)*180/pi;
band = settling_threshold * abs(initial_val - final_val);
idx = find(abs(Current2(:,4)*180/pi - final_val) > band, 1, 'last');
if isempty(idx)
    settling_time_roll = ta2(1);
else
    settling_time_roll = ta2(idx);
end
fprintf('基準ゲイン - ロール角整定時間: %.3f 秒\n', settling_time_roll);

% ピッチ角（Case2）
initial_val = eta0(2)*180/pi;
band = settling_threshold * abs(initial_val - final_val);
idx = find(abs(Current2(:,5)*180/pi - final_val) > band, 1, 'last');
if isempty(idx)
    settling_time_pitch = ta2(1);
else
    settling_time_pitch = ta2(idx);
end
fprintf('基準ゲイン - ピッチ角整定時間: %.3f 秒\n', settling_time_pitch);

%% 図を保存
saveas(figure(1), 'results_angle_comparison.png');
saveas(figure(2), 'results_angular_velocity_comparison.png');
fprintf('\n図を保存しました。\n');

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

    % PD 制御入力
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
