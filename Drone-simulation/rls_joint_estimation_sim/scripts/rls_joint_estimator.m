function [results] = rls_joint_estimator(y, t, omega_init, lambda, gamma, alpha_omega, P0)
% RLS_JOINT_ESTIMATOR A,B,C と omega を同時推定（交互 RLS + 勾配法）
%
% 入力:
%   y           - 観測信号 (N x 1)
%   t           - 時刻ベクトル (N x 1) [s]
%   omega_init  - omega の初期値 [rad/s]
%   lambda      - RLS 忘却係数 (0.95~0.999)
%   gamma       - omega 勾配法の学習率 (1e-4~1e-2)
%   alpha_omega - omega 平滑化係数 (0.01~0.1)
%   P0          - 共分散行列初期値 (3x3, diag([1e3,1e3,1e3]) 推奨)
%
% 出力:
%   results - 構造体
%     .theta_hist   : パラメータ履歴 [A, B, C] (N x 3)
%     .omega_hist   : omega 推定履歴 (N x 1) [rad/s]
%     .y_pred_hist  : 予測値履歴 (N x 1)
%     .error_hist   : 誤差履歴 (N x 1)
%     .P_hist       : 共分散トレース履歴 (N x 1)

N = length(y);

% 初期化
theta = [0; 0; 0];  % [A; B; C]
P = P0;
omega = omega_init;
omega_smoothed = omega_init;

% 履歴保存用
theta_hist = zeros(N, 3);
omega_hist = zeros(N, 1);
y_pred_hist = zeros(N, 1);
error_hist = zeros(N, 1);
P_hist = zeros(N, 1);

% 累積位相
phase = 0;

for k = 1:N
    % 現在の時刻ステップ
    dt = 0;
    if k > 1
        dt = t(k) - t(k-1);
    else
        dt = t(2) - t(1);  % 最初のステップは次のステップから推定
    end
    
    % 位相の更新
    phase = phase + omega_smoothed * dt;
    
    % 回帰ベクトル (A, B, C に対応)
    phi = [sin(phase); cos(phase); 1];
    
    % 予測値
    y_pred = phi' * theta;
    
    % 誤差
    e = y(k) - y_pred;
    
    % ===== RLS 更新 (A, B, C) =====
    % ゲインベクトル
    K = (P * phi) / (lambda + phi' * P * phi);
    
    % パラメータ更新
    theta = theta + K * e;
    
    % 共分散行列更新
    P = (P - K * phi' * P) / lambda;
    
    % ===== 勾配法による omega 更新 =====
    % 誤差の omega に対する偏微分
    % e = y - (A*sin(phase) + B*cos(phase) + C)
    % de/dω = -A*cos(phase)*t - B*(-sin(phase))*t = -A*cos(phase)*t + B*sin(phase)*t
    % 実際には累積位相なので、phase = Σ(ω*dt) より
    % dphase/dω ≈ t (累積時間)
    % de/dω = -(A*cos(phase) - B*sin(phase)) * t_cumulative
    
    t_cumulative = t(k);
    
    % 勾配（符号に注意: 誤差を減らす方向）
    grad_omega = -e * (theta(1) * cos(phase) - theta(2) * sin(phase)) * t_cumulative;
    
    % omega 更新（勾配降下）
    omega_raw = omega - gamma * grad_omega;
    
    % クリッピング（物理的に妥当な範囲）
    omega_raw = max(0.1, min(20, omega_raw));  % [0.1, 20] rad/s
    
    % 平滑化
    omega_smoothed = (1 - alpha_omega) * omega_smoothed + alpha_omega * omega_raw;
    omega = omega_smoothed;
    
    % 履歴保存
    theta_hist(k, :) = theta';
    omega_hist(k) = omega;
    y_pred_hist(k) = y_pred;
    error_hist(k) = e;
    P_hist(k) = trace(P);
end

% 結果構造体
results.theta_hist = theta_hist;
results.omega_hist = omega_hist;
results.y_pred_hist = y_pred_hist;
results.error_hist = error_hist;
results.P_hist = P_hist;

end
