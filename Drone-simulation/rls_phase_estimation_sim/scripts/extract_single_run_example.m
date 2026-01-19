% extract_single_run_example.m - Extract and plot a single representative run
clear; close all; clc;

% Setup paths
results_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);
figures_dir = fullfile(results_dir, 'figures_v2');

if ~exist(figures_dir, 'dir')
    mkdir(figures_dir);
end

% Generate a representative signal and run RLS estimator
fprintf('Generating representative signal...\n');

% Parameters (best case from sweeps)
fs = 100;
duration = 60;
f_true = 0.8;
decay = 0.01;
noise_std = 0.01;

% Generate signal
dt = 1/fs;
t = (0:dt:duration)';
amp = exp(-decay * t);
y = amp .* sin(2*pi*f_true*t) + noise_std * randn(size(t));

% RLS parameters (best from sweeps)
params = struct();
params.lambda = 0.995;
params.P0 = 1e3 * eye(3);
params.theta0 = zeros(3,1);
params.initial_omega = 0.6;  % 25% error
params.phase_buffer_size = 300;
params.phase_update_interval = 100;  % More frequent updates
params.omega_alpha = 0.2;  % Faster convergence (best from sweep 2)
params.phase_corr_enabled = true;
params.phase_corr_thresh = 10.0;
params.min_cov = 1e-6;
params.max_cov = 1e8;

fprintf('Running RLS phase estimator...\n');
logs = rls_phase_estimator(y, t, params);

% Calculate metrics
est_freq_clean = logs.est_freq;
est_freq_clean(isnan(est_freq_clean)) = params.initial_omega;

% Find convergence time (within 2%)
tolerance = 0.02 * f_true;
converged_idx = find(abs(est_freq_clean - f_true) < tolerance, 1, 'first');
if ~isempty(converged_idx)
    conv_time = t(converged_idx);
else
    conv_time = NaN;
end

% Calculate steady state metrics (last 10s)
steady_start_idx = find(t >= (duration - 10), 1, 'first');
steady_vals = est_freq_clean(steady_start_idx:end);
steady_error = mean(steady_vals - f_true);
steady_std = std(steady_vals);

fprintf('\nResults:\n');
fprintf('  Convergence time: %.2f s\n', conv_time);
fprintf('  Steady error: %.6f Hz (%.4f%%)\n', steady_error, 100*abs(steady_error)/f_true);
fprintf('  Steady std: %.6f Hz\n', steady_std);

% Create comprehensive plot
fprintf('Creating detailed plot...\n');

figure('Position', [50 50 1400 1000]);

% Subplot 1: Signal and prediction
subplot(4,1,1)
plot(t, y, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
hold on;
plot(t, logs.ypred, 'r-', 'LineWidth', 1.5);
hold off;
grid on;
xlabel('Time [s]', 'FontSize', 11);
ylabel('Amplitude', 'FontSize', 11);
legend('Observation', 'RLS Prediction', 'Location', 'best', 'FontSize', 10);
title('(a) Observed Signal vs RLS Prediction', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 duration]);
set(gca, 'FontSize', 10);

% Subplot 2: Frequency estimation
subplot(4,1,2)
plot(t, est_freq_clean, 'b-', 'LineWidth', 1.5);
hold on;
plot([0 duration], [f_true f_true], 'r--', 'LineWidth', 2);
plot([0 duration], [f_true+tolerance f_true+tolerance], 'k:', 'LineWidth', 1);
plot([0 duration], [f_true-tolerance f_true-tolerance], 'k:', 'LineWidth', 1);
if ~isnan(conv_time)
    plot([conv_time conv_time], ylim, 'g--', 'LineWidth', 1.5);
    text(conv_time, f_true*1.1, sprintf(' Conv: %.2fs', conv_time), 'FontSize', 10, 'Color', 'g');
end
hold off;
grid on;
xlabel('Time [s]', 'FontSize', 11);
ylabel('Frequency [Hz]', 'FontSize', 11);
legend('Estimated', 'True Value', '±2% Tolerance', '', 'Convergence', 'Location', 'best', 'FontSize', 10);
title('(b) Frequency Estimation', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 duration]);
ylim([0.5 1.0]);
set(gca, 'FontSize', 10);

% Subplot 3: Parameters A, B, C
subplot(4,1,3)
plot(t, logs.theta_hist(1,:), 'b-', 'LineWidth', 1.5);
hold on;
plot(t, logs.theta_hist(2,:), 'r-', 'LineWidth', 1.5);
plot(t, logs.theta_hist(3,:), 'g-', 'LineWidth', 1.5);
hold off;
grid on;
xlabel('Time [s]', 'FontSize', 11);
ylabel('Parameter Value', 'FontSize', 11);
legend('A (sin coeff)', 'B (cos coeff)', 'C (DC)', 'Location', 'best', 'FontSize', 10);
title('(c) RLS Parameters (A, B, C)', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 duration]);
set(gca, 'FontSize', 10);

% Subplot 4: Estimation error
subplot(4,1,4)
error_hz = est_freq_clean - f_true;
plot(t, error_hz * 1000, 'Color', [0.8 0.4 0], 'LineWidth', 1.5);
hold on;
plot([0 duration], [0 0], 'k--', 'LineWidth', 1);
% Shade steady state region
patch([duration-10 duration duration duration-10], [min(error_hz)*1000 min(error_hz)*1000 max(error_hz)*1000 max(error_hz)*1000], ...
    [0.9 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
hold off;
grid on;
xlabel('Time [s]', 'FontSize', 11);
ylabel('Freq Error [mHz]', 'FontSize', 11);
title(sprintf('(d) Frequency Error (Steady: %.3f ± %.3f mHz)', steady_error*1000, steady_std*1000), ...
    'FontSize', 12, 'FontWeight', 'bold');
xlim([0 duration]);
set(gca, 'FontSize', 10);

sgtitle(sprintf('RLS Phase Estimation - Example Run (f=%.2fHz, init=%.2fHz, α=%.2f, noise=%.2f)', ...
    f_true, params.initial_omega, params.omega_alpha, noise_std), ...
    'FontSize', 14, 'FontWeight', 'bold');

% Save figure
saveas(gcf, fullfile(figures_dir, 'example_run_detailed.png'));
fprintf('Saved: example_run_detailed.png\n');

% Save data
save(fullfile(results_dir, 'example_run.mat'), 't', 'y', 'logs', 'params', 'f_true', ...
    'conv_time', 'steady_error', 'steady_std');
fprintf('Saved: example_run.mat\n');

fprintf('\n=== Example run analysis complete ===\n');
