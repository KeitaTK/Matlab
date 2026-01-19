%% Multi-frequency Interference Test for RLS Joint Estimation
% Tests RLS joint estimator (gradient descent method) against 
% multi-frequency signals: 0.8 Hz primary + 5 Hz interference

clc; clear; close all;

% Add parent directory to path for accessing generate_multi_frequency_sine
addpath('..');
addpath('../scripts');

%% Test configurations
configs = struct();

% Configuration 1: Default parameters (baseline)
configs(1).name = 'default';
configs(1).gamma = 5e-3;           % Learning rate
configs(1).alpha_omega = 0.10;     % Smoothing factor
configs(1).A_secondary = 0.30;     % 30% interference amplitude

% Configuration 2: Higher gamma (faster omega updates)
configs(2).name = 'high_gamma';
configs(2).gamma = 1e-2;           % 2x higher learning rate
configs(2).alpha_omega = 0.10;
configs(2).A_secondary = 0.30;

% Configuration 3: Lower gamma (more stable)
configs(3).name = 'low_gamma';
configs(3).gamma = 1e-3;           % 5x lower learning rate
configs(3).alpha_omega = 0.05;     % Lower smoothing too
configs(3).A_secondary = 0.30;

% Configuration 4: Strong interference (50% amplitude)
configs(4).name = 'strong_interference';
configs(4).gamma = 5e-3;
configs(4).alpha_omega = 0.10;
configs(4).A_secondary = 0.50;     % Stronger interference

%% Common parameters
T = 60;                  % Duration [s]
fs = 100;                % Sampling frequency [Hz]
f_target = 0.8;          % Target frequency [Hz]
f_interference = 5.0;    % Interference frequency [Hz]
A_primary = 1.0;         % Primary amplitude
decay_rate = 0.01;       % Decay rate
noise_std = 0.0;         % No additional noise

% RLS parameters
lambda = 0.995;          % Forgetting factor
P0 = diag([1e3, 1e3, 1e3]);  % Initial covariance
omega_init = 2*pi*0.5;   % Initial guess: 0.5 Hz

%% Run tests
results_summary = struct();

for cfg_idx = 1:length(configs)
    cfg = configs(cfg_idx);
    
    fprintf('\n=== Configuration %d/%d: %s ===\n', cfg_idx, length(configs), cfg.name);
    
    % Generate multi-frequency signal
    fprintf('Generating multi-frequency signal...\n');
    signal_file = sprintf('../data/multi_freq_%s.csv', cfg.name);
    [y, t, true_omega, ~, ~] = generate_multi_frequency_sine(...
        f_target, f_interference, ...
        A_primary, cfg.A_secondary, ...
        decay_rate, noise_std, T, fs, signal_file);
    
    % Run RLS joint estimator
    fprintf('Running RLS joint estimator...\n');
    results = rls_joint_estimator(y, t, omega_init, lambda, cfg.gamma, cfg.alpha_omega, P0);
    
    % Calculate metrics
    conv_threshold = 0.02 * 2*pi;  % 2% of 2*pi*f_target
    conv_idx = find(abs(results.omega_hist - true_omega(end)) < conv_threshold, 1);
    if ~isempty(conv_idx)
        conv_time = t(conv_idx);
    else
        conv_time = NaN;
    end
    
    % Steady-state analysis (last 20 seconds)
    steady_start_idx = find(t >= T - 20, 1);
    steady_omega = results.omega_hist(steady_start_idx:end);
    steady_error = mean(steady_omega) - true_omega(end);
    steady_std = std(steady_omega);
    
    % Calculate error in Hz
    steady_error_hz = steady_error / (2*pi);
    steady_std_hz = steady_std / (2*pi);
    
    % Analyze dominant interference frequency component
    % FFT of omega estimation error
    omega_error = results.omega_hist(steady_start_idx:end) - true_omega(steady_start_idx:end);
    N_fft = length(omega_error);
    omega_error_fft = fft(omega_error);
    f_fft = (0:N_fft-1) * fs / N_fft;
    power = abs(omega_error_fft(1:floor(N_fft/2)+1)).^2;
    [~, max_idx] = max(power(2:end));  % Exclude DC
    dominant_freq = f_fft(max_idx + 1);
    
    % Store results
    results_summary(cfg_idx).name = cfg.name;
    results_summary(cfg_idx).gamma = cfg.gamma;
    results_summary(cfg_idx).alpha_omega = cfg.alpha_omega;
    results_summary(cfg_idx).A_secondary = cfg.A_secondary;
    results_summary(cfg_idx).conv_time = conv_time;
    results_summary(cfg_idx).steady_error_hz = steady_error_hz;
    results_summary(cfg_idx).steady_std_hz = steady_std_hz;
    results_summary(cfg_idx).dominant_freq = dominant_freq;
    results_summary(cfg_idx).results = results;
    results_summary(cfg_idx).y = y;
    results_summary(cfg_idx).t = t;
    results_summary(cfg_idx).true_omega = true_omega;
    
    % Print results
    fprintf('Results:\n');
    fprintf('  Convergence time: %.2f s\n', conv_time);
    fprintf('  Steady error: %.6f Hz (%.4f%%)\n', steady_error_hz, abs(steady_error_hz)/f_target*100);
    fprintf('  Steady std: %.6f Hz\n', steady_std_hz);
    fprintf('  Dominant interference freq: %.2f Hz\n', dominant_freq);
end

%% Summary comparison
fprintf('\n=== Summary Comparison ===\n');
fprintf('%20s | %12s | %15s | %15s | %16s\n', ...
    'Config', 'Conv [s]', 'Error [Hz]', 'Std [Hz]', 'Interf Freq [Hz]');
fprintf('%s\n', repmat('-', 1, 85));
for cfg_idx = 1:length(results_summary)
    r = results_summary(cfg_idx);
    fprintf('%20s | %12.2f | %15.6f | %15.6f | %16.2f\n', ...
        r.name, r.conv_time, r.steady_error_hz, r.steady_std_hz, r.dominant_freq);
end

%% Create comparison plots
figure('Position', [100, 100, 1400, 900]);

% Plot 1: Omega estimation over time
subplot(2, 2, 1);
hold on; grid on;
colors = lines(length(configs));
for cfg_idx = 1:length(results_summary)
    r = results_summary(cfg_idx);
    plot(r.t, r.results.omega_hist/(2*pi), 'Color', colors(cfg_idx, :), ...
        'LineWidth', 1.5, 'DisplayName', r.name);
end
plot(results_summary(1).t, results_summary(1).true_omega/(2*pi), 'k--', ...
    'LineWidth', 2, 'DisplayName', 'True');
xlabel('Time [s]');
ylabel('Frequency [Hz]');
title('Frequency Estimation (Multi-frequency Signal)');
legend('Location', 'best');
xlim([0, T]);

% Plot 2: Estimation error over time
subplot(2, 2, 2);
hold on; grid on;
for cfg_idx = 1:length(results_summary)
    r = results_summary(cfg_idx);
    error = (r.results.omega_hist - r.true_omega) / (2*pi);
    plot(r.t, error, 'Color', colors(cfg_idx, :), ...
        'LineWidth', 1.5, 'DisplayName', r.name);
end
yline(0, 'k--', 'LineWidth', 1);
xlabel('Time [s]');
ylabel('Error [Hz]');
title('Frequency Estimation Error');
legend('Location', 'best');
xlim([0, T]);

% Plot 3: Convergence time comparison
subplot(2, 2, 3);
conv_times = [results_summary.conv_time];
bar_colors = colors;
b = bar(conv_times);
b.FaceColor = 'flat';
b.CData = bar_colors;
set(gca, 'XTickLabel', {results_summary.name}, 'XTickLabelRotation', 45);
ylabel('Convergence Time [s]');
title('Convergence Time Comparison');
grid on;

% Plot 4: Steady-state error comparison
subplot(2, 2, 4);
steady_errors = abs([results_summary.steady_error_hz]);
b = bar(steady_errors);
b.FaceColor = 'flat';
b.CData = bar_colors;
set(gca, 'XTickLabel', {results_summary.name}, 'XTickLabelRotation', 45);
ylabel('|Steady Error| [Hz]');
title('Steady-State Error Comparison');
grid on;

sgtitle('RLS Joint Estimation: Multi-frequency Interference Test', 'FontSize', 14, 'FontWeight', 'bold');

% Save figure
fig_dir = '../results/figures_multi_freq';
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end
saveas(gcf, fullfile(fig_dir, 'joint_multi_freq_comparison.png'));

%% Create detailed plot for each configuration
for cfg_idx = 1:length(results_summary)
    r = results_summary(cfg_idx);
    
    figure('Position', [100, 100, 1400, 900]);
    
    % Input signal
    subplot(3, 2, 1);
    plot(r.t, r.y, 'b-', 'LineWidth', 1);
    xlabel('Time [s]');
    ylabel('Amplitude');
    title(sprintf('Input Signal (%s)', r.name));
    grid on;
    xlim([0, min(10, T)]);  % Show first 10 seconds
    
    % Signal spectrum
    subplot(3, 2, 2);
    N_fft = length(r.y);
    Y_fft = fft(r.y);
    f_fft = (0:N_fft-1) * fs / N_fft;
    power = abs(Y_fft(1:floor(N_fft/2)+1)).^2;
    plot(f_fft(1:floor(N_fft/2)+1), 10*log10(power), 'b-', 'LineWidth', 1.5);
    xlabel('Frequency [Hz]');
    ylabel('Power [dB]');
    title('Input Signal Spectrum');
    grid on;
    xlim([0, 10]);
    
    % Omega estimation
    subplot(3, 2, 3);
    hold on; grid on;
    plot(r.t, r.results.omega_hist/(2*pi), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Estimated');
    plot(r.t, r.true_omega/(2*pi), 'r--', 'LineWidth', 2, 'DisplayName', 'True');
    xlabel('Time [s]');
    ylabel('Frequency [Hz]');
    title('Frequency Estimation');
    legend('Location', 'best');
    xlim([0, T]);
    
    % Omega error
    subplot(3, 2, 4);
    error = (r.results.omega_hist - r.true_omega) / (2*pi);
    plot(r.t, error, 'b-', 'LineWidth', 1.5);
    yline(0, 'k--', 'LineWidth', 1);
    xlabel('Time [s]');
    ylabel('Error [Hz]');
    title('Frequency Estimation Error');
    grid on;
    xlim([0, T]);
    
    % Theta parameters
    subplot(3, 2, 5);
    hold on; grid on;
    plot(r.t, r.results.theta_hist(:,1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'A');
    plot(r.t, r.results.theta_hist(:,2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'B');
    plot(r.t, r.results.theta_hist(:,3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'C');
    xlabel('Time [s]');
    ylabel('Parameter Value');
    title('RLS Parameters (A, B, C)');
    legend('Location', 'best');
    xlim([0, T]);
    
    % Prediction error
    subplot(3, 2, 6);
    plot(r.t, r.results.error_hist, 'b-', 'LineWidth', 1);
    xlabel('Time [s]');
    ylabel('Prediction Error');
    title('RLS Prediction Error');
    grid on;
    xlim([0, T]);
    
    sgtitle(sprintf('RLS Joint Estimation: %s (γ=%.0e, α_ω=%.2f, A_{sec}=%.2f)', ...
        r.name, r.gamma, r.alpha_omega, r.A_secondary), ...
        'FontSize', 14, 'FontWeight', 'bold');
    
    % Save figure
    saveas(gcf, fullfile(fig_dir, sprintf('joint_multi_freq_%s.png', r.name)));
end

%% Save results
save('../results/multi_freq_joint_results.mat', 'results_summary', 'configs');

fprintf('\n=== Multi-frequency test complete ===\n');
