% SWEEP_PARAMS_JOINT パラメータスイープによる感度解析
% gamma, noise, 初期omega, decay の影響を評価
clear; close all; clc;

% ========== 固定パラメータ ==========
fs = 100;
duration = 60;
f_true = 0.8;
A_true = 1.0;
lambda = 0.995;
P0 = diag([1e3, 1e3, 1e3]);

% ========== スイープ1: gamma (学習率) ==========
fprintf('=== Sweep 1: gamma (学習率) ===\n');
gamma_values = [1e-3, 5e-3, 1e-2, 5e-2, 1e-1];
decay = 0.01;
noise_amp = 0.0;
omega_init = 2*pi*0.6;
alpha_omega = 0.1;

for i = 1:length(gamma_values)
    gamma = gamma_values(i);
    fprintf('gamma = %.1e ... ', gamma);
    
    [t, y, true_params] = generate_damped_sine(fs, duration, f_true, A_true, decay, noise_amp);
    results = rls_joint_estimator(y, t, omega_init, lambda, gamma, alpha_omega, P0);
    
    freq_est = results.omega_hist / (2*pi);
    converged_idx = find(abs(freq_est - f_true) < 0.02*f_true, 1, 'first');
    if isempty(converged_idx)
        conv_time = NaN;
    else
        conv_time = t(converged_idx);
    end
    
    steady_idx = find(t >= duration - 10);
    steady_error = mean(freq_est(steady_idx) - f_true);
    
    fprintf('収束時間=%.2f s, 定常誤差=%.4f Hz\n', conv_time, steady_error);
    
    % 結果保存
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    filename = sprintf('sweep_gamma_%.1e_%s', gamma, timestamp);
    save_dir = fullfile('..', 'results');
    save(fullfile(save_dir, [filename '.mat']), 'results', 'true_params', 't', 'y', 'gamma', 'conv_time', 'steady_error');
    plot_results_joint(results, t, y, true_params, conv_time, steady_error, 0, fullfile(save_dir, filename));
end

% ========== スイープ2: ノイズ振幅 ==========
fprintf('\n=== Sweep 2: ノイズ振幅 ===\n');
noise_values = [0.0, 0.01, 0.05, 0.1, 0.2];
gamma = 5e-2;

for i = 1:length(noise_values)
    noise_amp = noise_values(i);
    fprintf('noise_amp = %.2f ... ', noise_amp);
    
    [t, y, true_params] = generate_damped_sine(fs, duration, f_true, A_true, decay, noise_amp);
    results = rls_joint_estimator(y, t, omega_init, lambda, gamma, alpha_omega, P0);
    
    freq_est = results.omega_hist / (2*pi);
    converged_idx = find(abs(freq_est - f_true) < 0.02*f_true, 1, 'first');
    if isempty(converged_idx)
        conv_time = NaN;
    else
        conv_time = t(converged_idx);
    end
    
    steady_idx = find(t >= duration - 10);
    steady_error = mean(freq_est(steady_idx) - f_true);
    steady_std = std(freq_est(steady_idx));
    
    fprintf('収束時間=%.2f s, 定常誤差=%.4f±%.4f Hz\n', conv_time, steady_error, steady_std);
    
    % 結果保存
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    filename = sprintf('sweep_noise_%.2f_%s', noise_amp, timestamp);
    save_dir = fullfile('..', 'results');
    save(fullfile(save_dir, [filename '.mat']), 'results', 'true_params', 't', 'y', 'noise_amp', 'conv_time', 'steady_error', 'steady_std');
    plot_results_joint(results, t, y, true_params, conv_time, steady_error, steady_std, fullfile(save_dir, filename));
end

% ========== スイープ3: 初期 omega ==========
fprintf('\n=== Sweep 3: 初期 omega ===\n');
omega_init_values = [0.4, 0.5, 0.6, 0.7, 1.0, 1.2];  % [Hz]
noise_amp = 0.0;

for i = 1:length(omega_init_values)
    omega_init = 2*pi*omega_init_values(i);
    fprintf('omega_init = %.1f Hz ... ', omega_init_values(i));
    
    [t, y, true_params] = generate_damped_sine(fs, duration, f_true, A_true, decay, noise_amp);
    results = rls_joint_estimator(y, t, omega_init, lambda, gamma, alpha_omega, P0);
    
    freq_est = results.omega_hist / (2*pi);
    converged_idx = find(abs(freq_est - f_true) < 0.02*f_true, 1, 'first');
    if isempty(converged_idx)
        conv_time = NaN;
    else
        conv_time = t(converged_idx);
    end
    
    steady_idx = find(t >= duration - 10);
    steady_error = mean(freq_est(steady_idx) - f_true);
    
    fprintf('収束時間=%.2f s, 定常誤差=%.4f Hz\n', conv_time, steady_error);
    
    % 結果保存
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    filename = sprintf('sweep_ominit_%.1f_%s', omega_init_values(i), timestamp);
    save_dir = fullfile('..', 'results');
    save(fullfile(save_dir, [filename '.mat']), 'results', 'true_params', 't', 'y', 'omega_init', 'conv_time', 'steady_error');
    plot_results_joint(results, t, y, true_params, conv_time, steady_error, 0, fullfile(save_dir, filename));
end

% ========== スイープ4: decay (減衰率) ==========
fprintf('\n=== Sweep 4: decay (減衰率) ===\n');
decay_values = [0.001, 0.005, 0.01, 0.02, 0.05];
omega_init = 2*pi*0.6;

for i = 1:length(decay_values)
    decay = decay_values(i);
    fprintf('decay = %.3f ... ', decay);
    
    [t, y, true_params] = generate_damped_sine(fs, duration, f_true, A_true, decay, noise_amp);
    results = rls_joint_estimator(y, t, omega_init, lambda, gamma, alpha_omega, P0);
    
    freq_est = results.omega_hist / (2*pi);
    converged_idx = find(abs(freq_est - f_true) < 0.02*f_true, 1, 'first');
    if isempty(converged_idx)
        conv_time = NaN;
    else
        conv_time = t(converged_idx);
    end
    
    steady_idx = find(t >= duration - 10);
    steady_error = mean(freq_est(steady_idx) - f_true);
    
    fprintf('収束時間=%.2f s, 定常誤差=%.4f Hz\n', conv_time, steady_error);
    
    % 結果保存
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    filename = sprintf('sweep_decay_%.3f_%s', decay, timestamp);
    save_dir = fullfile('..', 'results');
    save(fullfile(save_dir, [filename '.mat']), 'results', 'true_params', 't', 'y', 'decay', 'conv_time', 'steady_error');
    plot_results_joint(results, t, y, true_params, conv_time, steady_error, 0, fullfile(save_dir, filename));
end

fprintf('\n=== スイープ完了 ===\n');
