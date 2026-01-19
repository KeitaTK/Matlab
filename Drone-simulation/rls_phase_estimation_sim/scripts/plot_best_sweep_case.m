% plot_best_sweep_case.m - Plot the best case from actual sweep results
clear; close all; clc;

% Load one of the best individual sweep results
results_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
figures_dir = fullfile(results_dir, 'figures_v2');

if ~exist(figures_dir, 'dir')
    mkdir(figures_dir);
end

% Load the best alpha case (alpha=0.05 had best balance)
csv_file = fullfile(results_dir, 'v2_alpha_0.05.csv');

if ~exist(csv_file, 'file')
    fprintf('Error: CSV file not found: %s\n', csv_file);
    return;
end

fprintf('Loading data from: %s\n', csv_file);
data = readtable(csv_file);

% Extract columns
t = data.Time_s;
y = data.Signal;
ypred = data.Predicted;
est_freq = data.EstFreq_Hz;
error_val = data.Error;

% For theta values, check if they exist
if ismember('theta_A', data.Properties.VariableNames)
    theta_A = data.theta_A;
    theta_B = data.theta_B;
    theta_C = data.theta_C;
else
    % Reconstruct from prediction if not available
    theta_A = nan(size(t));
    theta_B = nan(size(t));
    theta_C = nan(size(t));
end

% Clean NaN values in est_freq
est_freq_clean = est_freq;
est_freq_clean(isnan(est_freq_clean)) = 0.6;  % Initial value

% Parameters (from filename)
f_true = 0.8;
alpha_omega = 0.05;

% Calculate metrics
tolerance = 0.02 * f_true;
converged_idx = find(abs(est_freq_clean - f_true) < tolerance, 1, 'first');
if ~isempty(converged_idx)
    conv_time = t(converged_idx);
else
    conv_time = NaN;
end

% Steady state (last 10s)
duration = max(t);
steady_start_idx = find(t >= (duration - 10), 1, 'first');
steady_vals = est_freq_clean(steady_start_idx:end);
steady_error = mean(steady_vals - f_true);
steady_std = std(steady_vals);

fprintf('\nResults:\n');
fprintf('  Convergence time: %.2f s\n', conv_time);
fprintf('  Steady error: %.6f Hz (%.4f%%)\n', steady_error, 100*abs(steady_error)/f_true);
fprintf('  Steady std: %.6f Hz (%.4f%%)\n', steady_std, 100*steady_std/f_true);

% Create comprehensive plot
fprintf('Creating detailed plot...\n');

figure('Position', [50 50 1400 1000]);

% Subplot 1: Signal and prediction
subplot(4,1,1)
plot(t, y, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
hold on;
plot(t, ypred, 'r-', 'LineWidth', 1.5);
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
    text(conv_time+1, f_true*1.05, sprintf('Conv: %.2fs', conv_time), 'FontSize', 10, 'Color', 'g', 'FontWeight', 'bold');
end
hold off;
grid on;
xlabel('Time [s]', 'FontSize', 11);
ylabel('Frequency [Hz]', 'FontSize', 11);
legend('Estimated', 'True Value', '±2% Tolerance', 'Location', 'best', 'FontSize', 10);
title('(b) Frequency Estimation', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0 duration]);
ylim([0.55 0.95]);
set(gca, 'FontSize', 10);

% Subplot 3: Parameters A, B, C (if available)
subplot(4,1,3)
if ~all(isnan(theta_A))
    plot(t, theta_A, 'b-', 'LineWidth', 1.5);
    hold on;
    plot(t, theta_B, 'r-', 'LineWidth', 1.5);
    plot(t, theta_C, 'g-', 'LineWidth', 1.5);
    hold off;
    legend('A (sin coeff)', 'B (cos coeff)', 'C (DC)', 'Location', 'best', 'FontSize', 10);
    title('(c) RLS Parameters (A, B, C)', 'FontSize', 12, 'FontWeight', 'bold');
else
    plot(t, est_freq_clean, 'b-', 'LineWidth', 1.5);
    hold on;
    plot([0 duration], [f_true f_true], 'r--', 'LineWidth', 2);
    hold off;
    legend('Estimated', 'True Value', 'Location', 'best', 'FontSize', 10);
    title('(c) Frequency Tracking', 'FontSize', 12, 'FontWeight', 'bold');
end
grid on;
xlabel('Time [s]', 'FontSize', 11);
ylabel('Value', 'FontSize', 11);
xlim([0 duration]);
set(gca, 'FontSize', 10);

% Subplot 4: Estimation error
subplot(4,1,4)
error_hz = est_freq_clean - f_true;
plot(t, error_hz * 1000, 'Color', [0.8 0.4 0], 'LineWidth', 1.5);
hold on;
plot([0 duration], [0 0], 'k--', 'LineWidth', 1);
% Shade steady state region
y_min = min(error_hz)*1000 - 1;
y_max = max(error_hz)*1000 + 1;
patch([duration-10 duration duration duration-10], [y_min y_min y_max y_max], ...
    [0.9 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
hold off;
grid on;
xlabel('Time [s]', 'FontSize', 11);
ylabel('Freq Error [mHz]', 'FontSize', 11);
title(sprintf('(d) Frequency Error (Steady: %.3f ± %.3f mHz)', steady_error*1000, steady_std*1000), ...
    'FontSize', 12, 'FontWeight', 'bold');
xlim([0 duration]);
ylim([y_min y_max]);
set(gca, 'FontSize', 10);

sgtitle(sprintf('RLS Phase Estimation - Best Case Example (f=%.1fHz, α_ω=%.2f)', ...
    f_true, alpha_omega), ...
    'FontSize', 14, 'FontWeight', 'bold');

% Save figure
saveas(gcf, fullfile(figures_dir, 'best_case_detailed.png'));
fprintf('Saved: best_case_detailed.png\n');

% Zoom into first 20 seconds
figure('Position', [50 50 1400 600]);

subplot(2,1,1)
idx_zoom = t <= 20;
plot(t(idx_zoom), est_freq_clean(idx_zoom), 'b-', 'LineWidth', 2);
hold on;
plot([0 20], [f_true f_true], 'r--', 'LineWidth', 2);
plot([0 20], [f_true+tolerance f_true+tolerance], 'k:', 'LineWidth', 1);
plot([0 20], [f_true-tolerance f_true-tolerance], 'k:', 'LineWidth', 1);
if ~isnan(conv_time) && conv_time <= 20
    plot([conv_time conv_time], ylim, 'g--', 'LineWidth', 2);
    text(conv_time+0.5, f_true*1.05, sprintf('Conv: %.2fs', conv_time), 'FontSize', 11, 'Color', 'g', 'FontWeight', 'bold');
end
hold off;
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Frequency [Hz]', 'FontSize', 12);
legend('Estimated', 'True Value', '±2% Tolerance', 'Location', 'best', 'FontSize', 11);
title('Frequency Estimation - Convergence Phase (First 20s)', 'FontSize', 13, 'FontWeight', 'bold');
xlim([0 20]);
ylim([0.55 0.95]);
set(gca, 'FontSize', 11);

subplot(2,1,2)
error_hz_zoom = error_hz(idx_zoom);
plot(t(idx_zoom), error_hz_zoom * 1000, 'Color', [0.8 0.4 0], 'LineWidth', 2);
hold on;
plot([0 20], [0 0], 'k--', 'LineWidth', 1.5);
plot([0 20], [tolerance tolerance]*1000, 'k:', 'LineWidth', 1);
plot([0 20], [-tolerance -tolerance]*1000, 'k:', 'LineWidth', 1);
if ~isnan(conv_time) && conv_time <= 20
    plot([conv_time conv_time], ylim, 'g--', 'LineWidth', 2);
end
hold off;
grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('Freq Error [mHz]', 'FontSize', 12);
title('Frequency Error - Convergence Phase', 'FontSize', 13, 'FontWeight', 'bold');
xlim([0 20]);
set(gca, 'FontSize', 11);

sgtitle(sprintf('RLS Phase Estimation - Convergence Detail (α_ω=%.2f)', alpha_omega), ...
    'FontSize', 14, 'FontWeight', 'bold');

saveas(gcf, fullfile(figures_dir, 'best_case_convergence.png'));
fprintf('Saved: best_case_convergence.png\n');

fprintf('\n=== Best case plotting complete ===\n');
