% RUN_SIM_JOINT 交互 RLS + 勾配法によるジョイント推定のシミュレーション実行
clear; close all; clc;

% ========== パラメータ設定 ==========
% 信号生成パラメータ
fs = 100;               % サンプリング周波数 [Hz]
duration = 60;          % 信号長 [s]
f_true = 0.8;           % 真の周波数 [Hz]
A_true = 1.0;           % 真の振幅
decay = 0.01;           % 減衰係数 (約 30 秒時定数)
noise_amp = 0.0;        % ノイズ振幅 (0: ノイズなし)

% RLS パラメータ
lambda = 0.995;         % 忘却係数
P0 = diag([1e3, 1e3, 1e3]);  % 共分散行列初期値

% omega 推定パラメータ
omega_init = 2*pi*0.6;  % omega 初期値 [rad/s] (0.6 Hz)
gamma = 5e-2;           % 勾配法学習率（増加）
alpha_omega = 0.1;      % omega 平滑化係数（増加）

% ========== 信号生成 ==========
fprintf('信号生成中...\n');
[t, y, true_params] = generate_damped_sine(fs, duration, f_true, A_true, decay, noise_amp);

% ========== RLS ジョイント推定実行 ==========
fprintf('RLS ジョイント推定実行中...\n');
tic;
results = rls_joint_estimator(y, t, omega_init, lambda, gamma, alpha_omega, P0);
elapsed = toc;
fprintf('推定完了 (%.2f 秒)\n', elapsed);

% ========== 評価指標計算 ==========
% 周波数推定値
freq_est = results.omega_hist / (2*pi);

% 収束時間（|freq_est - f_true| < 2% を満たす最初の時刻）
convergence_thresh = 0.02 * f_true;
converged_idx = find(abs(freq_est - f_true) < convergence_thresh, 1, 'first');
if ~isempty(converged_idx)
    convergence_time = t(converged_idx);
else
    convergence_time = NaN;
end

% 定常誤差（最後 10 秒）
steady_idx = find(t >= duration - 10);
if ~isempty(steady_idx)
    steady_freq_error_mean = mean(freq_est(steady_idx) - f_true);
    steady_freq_error_std = std(freq_est(steady_idx));
else
    steady_freq_error_mean = NaN;
    steady_freq_error_std = NaN;
end

% ========== 結果保存 ==========
% ファイル名（設定値を含める）
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename_base = sprintf('joint_f%.2f_dec%.3f_lam%.3f_gam%.1e_aomg%.2f_n%.2f_%s', ...
    f_true, decay, lambda, gamma, alpha_omega, noise_amp, timestamp);

% データ保存
save_dir = fullfile('..', 'results');
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

% MAT ファイル
save(fullfile(save_dir, [filename_base '.mat']), ...
    'results', 'true_params', 't', 'y', ...
    'lambda', 'gamma', 'alpha_omega', 'omega_init', 'P0', ...
    'convergence_time', 'steady_freq_error_mean', 'steady_freq_error_std');

% CSV ファイル
csv_data = [t, y, results.y_pred_hist, results.error_hist, ...
    results.theta_hist, results.omega_hist, freq_est];
csv_header = 't,y,y_pred,error,A_est,B_est,C_est,omega_est,freq_est';
csv_file = fullfile(save_dir, [filename_base '.csv']);
writematrix(csv_data, csv_file);
% ヘッダー追加（手動）
fid = fopen(csv_file, 'r');
content = fread(fid, '*char')';
fclose(fid);
fid = fopen(csv_file, 'w');
fprintf(fid, '%s\n', csv_header);
fprintf(fid, '%s', content);
fclose(fid);

fprintf('結果保存完了: %s\n', save_dir);

% ========== プロット ==========
fprintf('プロット生成中...\n');
plot_results_joint(results, t, y, true_params, ...
    convergence_time, steady_freq_error_mean, steady_freq_error_std, ...
    fullfile(save_dir, filename_base));

fprintf('=== シミュレーション完了 ===\n');
fprintf('収束時間: %.2f s\n', convergence_time);
fprintf('定常誤差: %.4f Hz (std: %.4f Hz)\n', steady_freq_error_mean, steady_freq_error_std);
