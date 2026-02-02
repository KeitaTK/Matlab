% 反復計算用パラメータ
max_iter = 100;
tol = 1e-6;
%% quad_simu3_compare_implicit.m
% クアッドコプター姿勢制御のパラメータ比較シミュレーション（陰解法版）
% 異なるPD制御ゲインでの応答性能を比較
% 平均加速度法（ニューマークβ法: γ=1/2, β=1/4）を使用

clear; close all;
% 反復計算用パラメータ（clear後に再定義）
max_iter = 100;
tol = 1e-6;

%% 1. 前処理（関数ハンドルの読み込み）
mat_dir = fullfile(pwd, 'mat_data');
load(fullfile(mat_dir, "M_matrix.mat"),  "fM");
load(fullfile(mat_dir, "CoriGrav.mat"),  "fCoriGrav");
load(fullfile(mat_dir, "R_BtoI.mat"),    "fR_BtoI");
load(fullfile(mat_dir, "Omega.mat"),     "fOmega");
load(fullfile(mat_dir, "P_BtoI.mat"),    "fP_BtoI");
load(fullfile(mat_dir, "P_ItoB.mat"),    "fP_ItoB");

%% 2. 定数設定（3軸異なる慣性モーメント）
J_xx = 0.010;   % X軸慣性モーメント [kg·m²]
J_yy = 0.015;   % Y軸慣性モーメント [kg·m²]
J_zz = 0.020;   % Z軸慣性モーメント [kg·m²]
m    = 0.7;     % 質量 [kg]
g    = 9.81;    % 重力加速度 [m/s²]

const = struct('J_xx', J_xx, 'J_yy', J_yy, 'J_zz', J_zz, 'm', m, 'g', g);

%% 3. 初期条件
X0      = [0; 0; 0];                    % 位置 [m]
dX0     = [0; 0; 0];                    % 速度 [m/s]
eta0    = [40; 20; 10] * pi/180;        % ロール・ピッチ・ヨー [rad]
deta0   = [0; 0; 0];                    % 角速度 [rad/s]

q0   = [X0; eta0];
dq0  = [dX0; deta0];


%% 4. 制御器設定（3ケース：低ゲイン・適切ゲイン・高ゲイン）
etad = [0; 0; 0];  % 目標角度（すべて0度）

% Case 1: 低ゲイン（収束が非常に遅い）
kp1 = [1.5; 1.5; 1.5];
kd1 = [0.5; 0.5; 0.5];
controller1 = struct('etad', etad, 'kp', kp1, 'kd', kd1);

% Case 2: 適切なゲイン（良好な応答）
kp2 = [6.0; 6.0; 6.0];
kd2 = [2.5; 2.5; 2.5];
controller2 = struct('etad', etad, 'kp', kp2, 'kd', kd2);

% Case 3: 高ゲイン（発散）
kp3 = [30.0; 30.0; 30.0];
kd3 = [0.2; 0.2; 0.2];
controller3 = struct('etad', etad, 'kp', kp3, 'kd', kd3);

%% 5. シミュレーション設定
h      = 0.01;        % 時間刻み [s]
t_end  = 30;          % 終了時刻 [s]
N      = t_end / h;   % ステップ数

% ニューマークβ法パラメータ
gamma = 0.5;
beta = 0.25;

% 発散検出パラメータ
max_angle = 90;       % 最大角度 [deg]
max_warnings = 3;     % 特異行列警告の最大回数
divergence_check_interval = 10; % 発散チェック間隔

fprintf('陰解法による比較シミュレーション開始...\n');
fprintf('  方法: 平均加速度法（ニューマークβ法, γ=%.2f, β=%.2f）\n', gamma, beta);
fprintf('  時間刻み: %.4f [s], 終了時刻: %.1f [s]\n\n', h, t_end);

fprintf('\nCase 1: 低ゲイン実行中（Kp=[%.1f, %.1f, %.1f], Kd=[%.1f, %.1f, %.1f]）...\n', ...
    kp1(1), kp1(2), kp1(3), kd1(1), kd1(2), kd1(3));
[t1, q1, dq1, ddq1] = run_implicit_simulation_with_divergence_check(q0, dq0, const, controller1, ...
                                               h, t_end, gamma, beta, max_iter, tol, ...
                                               fM, fCoriGrav, fR_BtoI, fP_BtoI, fP_ItoB, ...
                                               max_angle, max_warnings, divergence_check_interval);
fprintf('  → 完了（シミュレーション時間: %.2f秒）\n', t1(end));

fprintf('\nCase 2: 適切ゲイン実行中（Kp=[%.1f, %.1f, %.1f], Kd=[%.1f, %.1f, %.1f]）...\n', ...
    kp2(1), kp2(2), kp2(3), kd2(1), kd2(2), kd2(3));
[t2, q2, dq2, ddq2] = run_implicit_simulation_with_divergence_check(q0, dq0, const, controller2, ...
                                               h, t_end, gamma, beta, max_iter, tol, ...
                                               fM, fCoriGrav, fR_BtoI, fP_BtoI, fP_ItoB, ...
                                               max_angle, max_warnings, divergence_check_interval);
fprintf('  → 完了（シミュレーション時間: %.2f秒）\n', t2(end));

fprintf('\nCase 3: 高ゲイン実行中（Kp=[%.1f, %.1f, %.1f], Kd=[%.1f, %.1f, %.1f]）...\n', ...
    kp3(1), kp3(2), kp3(3), kd3(1), kd3(2), kd3(3));
[t3, q3, dq3, ddq3] = run_implicit_simulation_with_divergence_check(q0, dq0, const, controller3, ...
                                               h, t_end, gamma, beta, max_iter, tol, ...
                                               fM, fCoriGrav, fR_BtoI, fP_BtoI, fP_ItoB, ...
                                               max_angle, max_warnings, divergence_check_interval);
fprintf('  → 完了（シミュレーション時間: %.2f秒）\n', t3(end));
%% 8. Case 3: 高ゲイン
fprintf('\nすべてのシミュレーション完了\n\n');

%% 9. 結果プロット: 角度の比較
figure(1);
set(gcf, 'Position', [100, 100, 1200, 400]);

% ロール角
subplot(1,3,1)
plot(t1, q1(:,4)*180/pi, 'LineWidth', 2); hold on;
plot(t2, q2(:,4)*180/pi, 'LineWidth', 2);
plot(t3, q3(:,4)*180/pi, 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1);
xlabel('時刻 [s]', 'FontSize', 12);
ylabel('ロール角 φ [deg]', 'FontSize', 12);
title('ロール角応答の比較（陰解法）', 'FontSize', 14);
legend('低ゲイン', '適切ゲイン', '高ゲイン', '目標値', 'Location', 'best');
grid on;

% ピッチ角
subplot(1,3,2)
plot(t1, q1(:,5)*180/pi, 'LineWidth', 2); hold on;
plot(t2, q2(:,5)*180/pi, 'LineWidth', 2);
plot(t3, q3(:,5)*180/pi, 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1);
xlabel('時刻 [s]', 'FontSize', 12);
ylabel('ピッチ角 θ [deg]', 'FontSize', 12);
title('ピッチ角応答の比較（陰解法）', 'FontSize', 14);
legend('低ゲイン', '適切ゲイン', '高ゲイン', '目標値', 'Location', 'best');
grid on;

% ヨー角
subplot(1,3,3)
plot(t1, q1(:,6)*180/pi, 'LineWidth', 2); hold on;
plot(t2, q2(:,6)*180/pi, 'LineWidth', 2);
plot(t3, q3(:,6)*180/pi, 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1);
xlabel('時刻 [s]', 'FontSize', 12);
ylabel('ヨー角 ψ [deg]', 'FontSize', 12);
title('ヨー角応答の比較（陰解法）', 'FontSize', 14);
legend('低ゲイン', '適切ゲイン', '高ゲイン', '目標値', 'Location', 'best');
grid on;

% --- デバッグ用出力 ---
fprintf('\n=== シミュレーション結果 ===\n');
fprintf('\n【最大角度】\n');
fprintf('  ロール角: 低ゲイン=%.2f°, 適切ゲイン=%.2f°, 高ゲイン=%.2f°\n', ...
    max(abs(q1(:,4)*180/pi)), max(abs(q2(:,4)*180/pi)), max(abs(q3(:,4)*180/pi)));
fprintf('  ピッチ角: 低ゲイン=%.2f°, 適切ゲイン=%.2f°, 高ゲイン=%.2f°\n', ...
    max(abs(q1(:,5)*180/pi)), max(abs(q2(:,5)*180/pi)), max(abs(q3(:,5)*180/pi)));
fprintf('  ヨー角  : 低ゲイン=%.2f°, 適切ゲイン=%.2f°, 高ゲイン=%.2f°\n', ...
    max(abs(q1(:,6)*180/pi)), max(abs(q2(:,6)*180/pi)), max(abs(q3(:,6)*180/pi)));

fprintf('\n【終了時の角度】\n');
fprintf('  ロール角: 低ゲイン=%.2f°, 適切ゲイン=%.2f°, 高ゲイン=%.2f°\n', ...
    q1(end,4)*180/pi, q2(end,4)*180/pi, q3(end,4)*180/pi);
fprintf('  ピッチ角: 低ゲイン=%.2f°, 適切ゲイン=%.2f°, 高ゲイン=%.2f°\n', ...
    q1(end,5)*180/pi, q2(end,5)*180/pi, q3(end,5)*180/pi);
fprintf('  ヨー角  : 低ゲイン=%.2f°, 適切ゲイン=%.2f°, 高ゲイン=%.2f°\n', ...
    q1(end,6)*180/pi, q2(end,6)*180/pi, q3(end,6)*180/pi);

fprintf('\n【シミュレーション時間】\n');
fprintf('  低ゲイン=%.2f秒, 適切ゲイン=%.2f秒, 高ゲイン=%.2f秒\n', ...
    t1(end), t2(end), t3(end));

if any(isnan(q3(:))) || any(isinf(q3(:)))
    warning('高ゲインケースでNaNまたはInfが発生しています');
end


%% 10. 性能指標の計算
settling_threshold = 0.02; % 2%
fprintf('\n=== 性能評価（ロール角） ===\n');

% ロール角整定時間
final_val = 0;
initial_val = eta0(1)*180/pi;
band = settling_threshold * abs(initial_val - final_val);

% 各ケースの整定時間計算
idx1 = find(abs(q1(:,4)*180/pi - final_val) > band, 1, 'last');
idx2 = find(abs(q2(:,4)*180/pi - final_val) > band, 1, 'last');
idx3 = find(abs(q3(:,4)*180/pi - final_val) > band, 1, 'last');

if isempty(idx1)
    settling_time1 = t1(end);
else
    settling_time1 = t1(idx1);
end

if isempty(idx2)
    settling_time2 = t2(end);
else
    settling_time2 = t2(idx2);
end

if isempty(idx3)
    settling_time3 = t3(end);
else
    settling_time3 = t3(idx3);
end

fprintf('低ゲイン - 整定時間: %.3f 秒\n', settling_time1);
fprintf('適切ゲイン - 整定時間: %.3f 秒\n', settling_time2);
fprintf('高ゲイン - 整定時間: %.3f 秒\n', settling_time3);

%% 11. 図を保存（角度のみ）
saveas(figure(1), 'results_angle_comparison_implicit.png');
fprintf('\n図を保存しました（角度のみ）。\n');

%% ========== 補助関数 ==========

%% 発散検出機能付き陰解法シミュレーション実行関数
function [t_array, q_array, dq_array, ddq_array] = run_implicit_simulation_with_divergence_check(...
    q0, dq0, const, controller, h, t_end, gamma, beta, max_iter, tol, ...
    fM, fCoriGrav, fR_BtoI, fP_BtoI, fP_ItoB, max_angle, max_warnings, divergence_check_interval)
    
    N = t_end / h;
    
    % 結果保存用配列
    t_array   = zeros(N+1, 1);
    q_array   = zeros(N+1, 6);
    dq_array  = zeros(N+1, 6);
    ddq_array = zeros(N+1, 6);
    
    % 発散検出用カウンタ
    warning_count = 0;
    
    % 初期加速度を計算
    ddq0 = compute_acceleration(q0, dq0, const, fM, fCoriGrav, ...
                                fR_BtoI, fP_BtoI, fP_ItoB, controller);
    
    % 初期値を保存
    t_array(1)     = 0;
    q_array(1,:)   = q0.';
    dq_array(1,:)  = dq0.';
    ddq_array(1,:) = ddq0.';
    
    % 時間積分ループ
    for n = 1:N
        t_n   = (n-1) * h;
        t_np1 = n * h;
        
        % 現在の状態
        q_n   = q_array(n, :).';
        dq_n  = dq_array(n, :).';
        ddq_n = ddq_array(n, :).';
        
        % 発散チェック（一定間隔で実行）
        if mod(n, divergence_check_interval) == 0
            angles_deg = q_n(4:6) * 180/pi;
            if any(abs(angles_deg) > max_angle) || any(isnan(q_n)) || any(isinf(q_n))
                fprintf('  発散検出により時刻 t=%.3f で終了 (最大角度: %.1f度)\n', t_n, max(abs(angles_deg)));
                % 配列を現在のステップまでで切り詰める
                t_array = t_array(1:n);
                q_array = q_array(1:n, :);
                dq_array = dq_array(1:n, :);
                ddq_array = ddq_array(1:n, :);
                return;
            end
        end
        
        % 予測ステップ
        ddq_np1 = ddq_n;
        
        % ニュートン・ラフソン法による収束計算
        for iter = 1:max_iter
            % ニューマークβ法による速度・位置の計算
            dq_np1 = dq_n + h * ((1 - gamma) * ddq_n + gamma * ddq_np1);
            q_np1  = q_n + h * dq_n + h^2 * ((0.5 - beta) * ddq_n + beta * ddq_np1);
            
            % 外力・トルクの計算
            Q_np1 = compute_generalized_force(q_np1, dq_np1, controller, const, ...
                                              fR_BtoI, fP_BtoI, fP_ItoB);
            
            % 残差の計算
            M_np1  = compute_mass_matrix(q_np1, dq_np1, const, fM);
            CG_np1 = compute_coriolis_gravity(q_np1, dq_np1, const, fCoriGrav);
            
            residual = M_np1 * ddq_np1 + CG_np1 - Q_np1;
            
            % 収束判定
            if norm(residual) < tol
                break;
            end
            
            % ヤコビ行列（有効質量行列）
            J_eff = M_np1;
            
            % 特異行列チェック
            if rcond(J_eff) < eps
                warning_count = warning_count + 1;
                if warning_count >= max_warnings
                    fprintf('  特異行列警告が%d回に達したため時刻 t=%.3f で終了\n', max_warnings, t_np1);
                    % 配列を現在のステップまでで切り詰める
                    t_array = t_array(1:n);
                    q_array = q_array(1:n, :);
                    dq_array = dq_array(1:n, :);
                    ddq_array = ddq_array(1:n, :);
                    return;
                end
            end
            
            % 修正量の計算
            delta_ddq = -J_eff \ residual;
            
            % 加速度の更新
            ddq_np1 = ddq_np1 + delta_ddq;
        end
        
        % 収束しない場合の警告（簡潔に）
        if iter == max_iter && norm(residual) >= tol
            warning_count = warning_count + 1;
            if warning_count >= max_warnings
                fprintf('  収束失敗が%d回に達したため時刻 t=%.3f で終了\n', max_warnings, t_np1);
                % 配列を現在のステップまでで切り詰める
                t_array = t_array(1:n);
                q_array = q_array(1:n, :);
                dq_array = dq_array(1:n, :);
                ddq_array = ddq_array(1:n, :);
                return;
            end
        end
        
        % 結果を保存
        t_array(n+1)     = t_np1;
        q_array(n+1, :)  = q_np1.';
        dq_array(n+1, :) = dq_np1.';
        ddq_array(n+1, :) = ddq_np1.';
    end
end

%% 陰解法シミュレーション実行関数
%% 初期加速度の計算
function ddq = compute_acceleration(q, dq, const, fM, fCoriGrav, ...
                                    fR_BtoI, fP_BtoI, fP_ItoB, controller)
    % 質量行列
    M = compute_mass_matrix(q, dq, const, fM);
    
    % コリオリ・重力項
    CG = compute_coriolis_gravity(q, dq, const, fCoriGrav);
    
    % 外力
    Q = compute_generalized_force(q, dq, controller, const, ...
                                  fR_BtoI, fP_BtoI, fP_ItoB);
    
    % 加速度の計算
    ddq = M \ (-CG + Q);
end

%% 質量行列の計算
function M = compute_mass_matrix(q, dq, const, fM)
    X   = q(1:3);
    eta = q(4:6);
    dX   = dq(1:3);
    deta = dq(4:6);
    
    M = fM(X(1), X(2), X(3), eta(1), eta(2), eta(3), ...
           dX(1), dX(2), dX(3), deta(1), deta(2), deta(3), ...
           const.J_xx, const.J_yy, const.J_zz, const.m, const.g);
end

%% コリオリ・重力項の計算
function CG = compute_coriolis_gravity(q, dq, const, fCoriGrav)
    X   = q(1:3);
    eta = q(4:6);
    dX   = dq(1:3);
    deta = dq(4:6);
    
    CG = fCoriGrav(X(1), X(2), X(3), eta(1), eta(2), eta(3), ...
                   dX(1), dX(2), dX(3), deta(1), deta(2), deta(3), ...
                   const.J_xx, const.J_yy, const.J_zz, const.m, const.g);
end

%% 一般化力の計算（制御入力）
function Q = compute_generalized_force(q, dq, controller, const, ...
                                       fR_BtoI, fP_BtoI, fP_ItoB)
    X   = q(1:3);
    eta = q(4:6);
    dX   = dq(1:3);
    deta = dq(4:6);
    
    % 座標変換行列
    R_BtoI = fR_BtoI(eta(1), eta(2), eta(3));
    P_BtoI = fP_BtoI(eta(1), eta(2), eta(3));
    P_ItoB = fP_ItoB(eta(1), eta(2), eta(3));
    
    % 機体座標系での角度と角速度
    eta_b  = P_ItoB * eta;
    deta_b = P_ItoB * deta;
    
    % PD制御
    u = -controller.kp .* (eta_b - controller.etad) - controller.kd .* deta_b;
    
    % モーター入力への変換
    input_sim = u * 100/(1023-551);
    
    % モーター力・トルク
    motor_torq = 4 * diag([0.1425, 0.090, 1]) * ...
                 diag([0.02219, 0.02219, 0.0003533]) * input_sim;
    F_b   = [0; 0; 1] * 80 * 0.02219 * 4;
    Tau_b = motor_torq;
    
    % 慣性座標系への変換
    F_I   = R_BtoI * F_b;
    Tau_I = P_BtoI * Tau_b;
    
    Q = [F_I; Tau_I];
end
