% create_summary_figures.m - Create comprehensive summary figures
clear; close all; clc;

results_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
mat_file = fullfile(results_dir, 'v2_all_sweep_results.mat');
figures_dir = fullfile(results_dir, 'figures_v2');

if ~exist(figures_dir, 'dir')
    mkdir(figures_dir);
end

fprintf('Loading results...\n');
load(mat_file);

% Define colors
colors = lines(7);

%% Figure 1: Alpha_omega sweep comparison
fprintf('Creating Figure 1: Alpha_omega sweep...\n');
alpha_data = all_results{2};

figure('Position', [100 100 1200 800]);

subplot(3,1,1)
plot(alpha_data.values, alpha_data.conv_time, '-o', 'LineWidth', 2, 'MarkerSize', 8);
grid on;
xlabel('Alpha\_omega', 'FontSize', 12);
ylabel('Convergence Time [s]', 'FontSize', 12);
title('Convergence Time vs Alpha\_omega', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 11);

subplot(3,1,2)
plot(alpha_data.values, abs(alpha_data.steady_error) * 1000, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'Color', colors(2,:));
grid on;
xlabel('Alpha\_omega', 'FontSize', 12);
ylabel('|Steady Error| [mHz]', 'FontSize', 12);
title('Absolute Steady Error vs Alpha\_omega', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 11);

subplot(3,1,3)
plot(alpha_data.values, alpha_data.steady_std * 1000, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'Color', colors(3,:));
grid on;
xlabel('Alpha\_omega', 'FontSize', 12);
ylabel('Steady Std [mHz]', 'FontSize', 12);
title('Steady Standard Deviation vs Alpha\_omega', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 11);

saveas(gcf, fullfile(figures_dir, 'sweep_alpha_summary.png'));
fprintf('  Saved: sweep_alpha_summary.png\n');

%% Figure 2: Noise sweep comparison
fprintf('Creating Figure 2: Noise sweep...\n');
noise_data = all_results{3};

figure('Position', [100 100 1200 800]);

subplot(3,1,1)
plot(noise_data.values, noise_data.conv_time, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'Color', colors(4,:));
grid on;
xlabel('Noise Std', 'FontSize', 12);
ylabel('Convergence Time [s]', 'FontSize', 12);
title('Convergence Time vs Noise Level', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 11);

subplot(3,1,2)
plot(noise_data.values, abs(noise_data.steady_error) * 1000, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'Color', colors(5,:));
grid on;
xlabel('Noise Std', 'FontSize', 12);
ylabel('|Steady Error| [mHz]', 'FontSize', 12);
title('Absolute Steady Error vs Noise Level', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 11);

subplot(3,1,3)
semilogy(noise_data.values, noise_data.steady_std * 1000, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'Color', colors(6,:));
grid on;
xlabel('Noise Std', 'FontSize', 12);
ylabel('Steady Std [mHz] (log)', 'FontSize', 12);
title('Steady Standard Deviation vs Noise Level', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 11);

saveas(gcf, fullfile(figures_dir, 'sweep_noise_summary.png'));
fprintf('  Saved: sweep_noise_summary.png\n');

%% Figure 3: Initial omega sweep comparison
fprintf('Creating Figure 3: Initial omega sweep...\n');
init_omega_data = all_results{4};

figure('Position', [100 100 1200 800]);

subplot(3,1,1)
plot(init_omega_data.values, init_omega_data.conv_time, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'Color', colors(1,:));
grid on;
xlabel('Initial Omega [Hz]', 'FontSize', 12);
ylabel('Convergence Time [s]', 'FontSize', 12);
title('Convergence Time vs Initial Frequency Guess', 'FontSize', 14, 'FontWeight', 'bold');
hold on;
plot([0.8 0.8], ylim, '--r', 'LineWidth', 1.5);
text(0.8, max(init_omega_data.conv_time)*0.9, ' True Value (0.8Hz)', 'FontSize', 10, 'Color', 'r');
hold off;
set(gca, 'FontSize', 11);

subplot(3,1,2)
plot(init_omega_data.values, abs(init_omega_data.steady_error) * 1000, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'Color', colors(2,:));
grid on;
xlabel('Initial Omega [Hz]', 'FontSize', 12);
ylabel('|Steady Error| [mHz]', 'FontSize', 12);
title('Absolute Steady Error vs Initial Frequency Guess', 'FontSize', 14, 'FontWeight', 'bold');
hold on;
plot([0.8 0.8], ylim, '--r', 'LineWidth', 1.5);
hold off;
set(gca, 'FontSize', 11);

subplot(3,1,3)
plot(init_omega_data.values, init_omega_data.steady_std * 1000, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'Color', colors(3,:));
grid on;
xlabel('Initial Omega [Hz]', 'FontSize', 12);
ylabel('Steady Std [mHz]', 'FontSize', 12);
title('Steady Standard Deviation vs Initial Frequency Guess', 'FontSize', 14, 'FontWeight', 'bold');
hold on;
plot([0.8 0.8], ylim, '--r', 'LineWidth', 1.5);
hold off;
set(gca, 'FontSize', 11);

saveas(gcf, fullfile(figures_dir, 'sweep_init_omega_summary.png'));
fprintf('  Saved: sweep_init_omega_summary.png\n');

%% Figure 4: Decay sweep comparison
fprintf('Creating Figure 4: Decay sweep...\n');
decay_data = all_results{5};

figure('Position', [100 100 1200 800]);

subplot(3,1,1)
plot(decay_data.values, decay_data.conv_time, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'Color', colors(4,:));
grid on;
xlabel('Decay Coefficient [1/s]', 'FontSize', 12);
ylabel('Convergence Time [s]', 'FontSize', 12);
title('Convergence Time vs Signal Decay', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 11);

subplot(3,1,2)
semilogy(decay_data.values, abs(decay_data.steady_error) * 1000, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'Color', colors(5,:));
grid on;
xlabel('Decay Coefficient [1/s]', 'FontSize', 12);
ylabel('|Steady Error| [mHz] (log)', 'FontSize', 12);
title('Absolute Steady Error vs Signal Decay', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 11);

subplot(3,1,3)
semilogy(decay_data.values, decay_data.steady_std * 1000, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'Color', colors(6,:));
grid on;
xlabel('Decay Coefficient [1/s]', 'FontSize', 12);
ylabel('Steady Std [mHz] (log)', 'FontSize', 12);
title('Steady Standard Deviation vs Signal Decay', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 11);

saveas(gcf, fullfile(figures_dir, 'sweep_decay_summary.png'));
fprintf('  Saved: sweep_decay_summary.png\n');

%% Figure 5: Overall comparison across all sweeps
fprintf('Creating Figure 5: Overall comparison...\n');

figure('Position', [100 100 1400 900]);

sweep_names = {'Gamma', 'Alpha', 'Noise', 'Init Omega', 'Decay'};
param_labels = {'gamma', 'alpha\_omega', 'noise\_std', 'initial\_omega', 'decay'};

for i = 1:5
    data = all_results{i};
    
    subplot(3,5,i)
    bar(data.conv_time);
    ylabel('Conv Time [s]', 'FontSize', 9);
    title(sweep_names{i}, 'FontSize', 11, 'FontWeight', 'bold');
    set(gca, 'XTickLabel', arrayfun(@num2str, data.values, 'UniformOutput', false), 'FontSize', 8);
    xtickangle(45);
    grid on;
    
    subplot(3,5,i+5)
    bar(abs(data.steady_error) * 1000);
    ylabel('|Error| [mHz]', 'FontSize', 9);
    set(gca, 'XTickLabel', arrayfun(@num2str, data.values, 'UniformOutput', false), 'FontSize', 8);
    xtickangle(45);
    grid on;
    
    subplot(3,5,i+10)
    bar(data.steady_std * 1000);
    ylabel('Std [mHz]', 'FontSize', 9);
    xlabel(param_labels{i}, 'FontSize', 9);
    set(gca, 'XTickLabel', arrayfun(@num2str, data.values, 'UniformOutput', false), 'FontSize', 8);
    xtickangle(45);
    grid on;
end

sgtitle('RLS Phase Estimation - Comprehensive Parameter Sweep Results', 'FontSize', 16, 'FontWeight', 'bold');

saveas(gcf, fullfile(figures_dir, 'all_sweeps_comparison.png'));
fprintf('  Saved: all_sweeps_comparison.png\n');

%% Figure 6: Best parameters summary
fprintf('Creating Figure 6: Best parameters...\n');

figure('Position', [100 100 1000 600]);

% Extract best values
best_conv = zeros(5,1);
best_error = zeros(5,1);
best_std = zeros(5,1);
best_conv_params = cell(5,1);
best_error_params = cell(5,1);
best_std_params = cell(5,1);

for i = 1:5
    data = all_results{i};
    [best_conv(i), idx] = min(data.conv_time);
    best_conv_params{i} = sprintf('%s=%.3g', param_labels{i}, data.values(idx));
    
    [best_error(i), idx] = min(abs(data.steady_error));
    best_error_params{i} = sprintf('%s=%.3g', param_labels{i}, data.values(idx));
    
    [best_std(i), idx] = min(data.steady_std);
    best_std_params{i} = sprintf('%s=%.3g', param_labels{i}, data.values(idx));
end

subplot(1,3,1)
bar(best_conv);
ylabel('Best Convergence Time [s]', 'FontSize', 11);
title('Fastest Convergence', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'XTickLabel', sweep_names, 'FontSize', 10);
xtickangle(45);
grid on;
for i = 1:5
    text(i, best_conv(i), sprintf('  %.2fs', best_conv(i)), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
end

subplot(1,3,2)
bar(best_error * 1000);
ylabel('Best |Steady Error| [mHz]', 'FontSize', 11);
title('Lowest Error', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'XTickLabel', sweep_names, 'FontSize', 10);
xtickangle(45);
grid on;
for i = 1:5
    text(i, best_error(i)*1000, sprintf('  %.3f', best_error(i)*1000), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
end

subplot(1,3,3)
bar(best_std * 1000);
ylabel('Best Steady Std [mHz]', 'FontSize', 11);
title('Lowest Std', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'XTickLabel', sweep_names, 'FontSize', 10);
xtickangle(45);
grid on;
for i = 1:5
    text(i, best_std(i)*1000, sprintf('  %.3f', best_std(i)*1000), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
end

sgtitle('Best Performance Across Parameter Sweeps', 'FontSize', 14, 'FontWeight', 'bold');

saveas(gcf, fullfile(figures_dir, 'best_parameters_summary.png'));
fprintf('  Saved: best_parameters_summary.png\n');

fprintf('\n=== All summary figures created successfully ===\n');
fprintf('Figures saved to: %s\n', figures_dir);
