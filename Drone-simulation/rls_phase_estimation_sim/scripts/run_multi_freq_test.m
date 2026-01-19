% run_multi_freq_test.m - Test RLS phase estimation with multi-frequency signal
clear; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);
results_dir = fullfile(script_dir,'..','results');
if ~exist(results_dir,'dir'), mkdir(results_dir); end

% Test configurations
configs = struct();

% Config 1: Default parameters
configs(1).name = 'default';
configs(1).f_primary = 0.8;
configs(1).f_secondary = 5.0;
configs(1).A_primary = 1.0;
configs(1).A_secondary = 0.3;
configs(1).initial_omega = 0.6;
configs(1).omega_alpha = 0.1;
configs(1).phase_buffer_size = 300;
configs(1).phase_update_interval = 300;

% Config 2: Higher alpha (faster adaptation)
configs(2).name = 'high_alpha';
configs(2).f_primary = 0.8;
configs(2).f_secondary = 5.0;
configs(2).A_primary = 1.0;
configs(2).A_secondary = 0.3;
configs(2).initial_omega = 0.6;
configs(2).omega_alpha = 0.2;
configs(2).phase_buffer_size = 300;
configs(2).phase_update_interval = 100;

% Config 3: Lower alpha (more stable)
configs(3).name = 'low_alpha';
configs(3).f_primary = 0.8;
configs(3).f_secondary = 5.0;
configs(3).A_primary = 1.0;
configs(3).A_secondary = 0.3;
configs(3).initial_omega = 0.6;
configs(3).omega_alpha = 0.05;
configs(3).phase_buffer_size = 300;
configs(3).phase_update_interval = 300;

% Config 4: Strong interference
configs(4).name = 'strong_interference';
configs(4).f_primary = 0.8;
configs(4).f_secondary = 5.0;
configs(4).A_primary = 1.0;
configs(4).A_secondary = 0.5;  % Stronger interference
configs(4).initial_omega = 0.6;
configs(4).omega_alpha = 0.1;
configs(4).phase_buffer_size = 300;
configs(4).phase_update_interval = 300;

% Run all configurations
results = cell(length(configs), 1);

for i = 1:length(configs)
    cfg = configs(i);
    fprintf('\n=== Configuration %d/%d: %s ===\n', i, length(configs), cfg.name);
    
    % Generate signal
    fprintf('Generating multi-frequency signal...\n');
    outname = sprintf('multi_freq_%s.csv', cfg.name);
    sig = generate_multi_frequency_sine(...
        'fs', 100, 'duration', 60, ...
        'f_primary', cfg.f_primary, ...
        'f_secondary', cfg.f_secondary, ...
        'A_primary', cfg.A_primary, ...
        'A_secondary', cfg.A_secondary, ...
        'decay', 0.01, ...
        'noise_std', 0.01, ...
        'outname', outname);
    
    % Load data
    T = readtable(sig.path);
    t = T.Time_s;
    y = T.Signal;
    
    % RLS parameters
    params = struct();
    params.lambda = 0.995;
    params.P0 = 1e3 * eye(3);
    params.theta0 = zeros(3,1);
    params.initial_omega = cfg.initial_omega;
    params.phase_buffer_size = cfg.phase_buffer_size;
    params.phase_update_interval = cfg.phase_update_interval;
    params.omega_alpha = cfg.omega_alpha;
    params.phase_corr_enabled = true;
    params.phase_corr_thresh = 10.0;
    params.min_cov = 1e-6;
    params.max_cov = 1e8;
    
    % Run RLS estimator
    fprintf('Running RLS phase estimator...\n');
    logs = rls_phase_estimator(y, t, params);
    
    % Calculate metrics
    est_freq_clean = logs.est_freq;
    est_freq_clean(isnan(est_freq_clean)) = cfg.initial_omega;
    
    % Find convergence time
    f_true = cfg.f_primary;
    tolerance = 0.02 * f_true;
    converged_idx = find(abs(est_freq_clean - f_true) < tolerance, 1, 'first');
    if ~isempty(converged_idx)
        conv_time = t(converged_idx);
    else
        conv_time = NaN;
    end
    
    % Steady state metrics (last 10s)
    duration = max(t);
    steady_start_idx = find(t >= (duration - 10), 1, 'first');
    steady_vals = est_freq_clean(steady_start_idx:end);
    steady_error = mean(steady_vals - f_true);
    steady_std = std(steady_vals);
    
    % Calculate interference rejection (how much secondary freq bleeds in)
    % Check if estimated frequency oscillates at secondary frequency
    freq_diff = diff(est_freq_clean);
    freq_diff_fft = fft(freq_diff);
    freq_axis = (0:length(freq_diff)-1) * (100/length(freq_diff));
    [~, peak_idx] = max(abs(freq_diff_fft(2:floor(length(freq_diff_fft)/2))));
    dominant_interference_freq = freq_axis(peak_idx+1);
    
    % Store results
    result = struct();
    result.config = cfg;
    result.conv_time = conv_time;
    result.steady_error = steady_error;
    result.steady_std = steady_std;
    result.dominant_interference_freq = dominant_interference_freq;
    result.logs = logs;
    result.signal = sig;
    result.t = t;
    result.y = y;
    result.est_freq = est_freq_clean;
    
    results{i} = result;
    
    fprintf('Results:\n');
    fprintf('  Convergence time: %.2f s\n', conv_time);
    fprintf('  Steady error: %.6f Hz (%.4f%%)\n', steady_error, 100*abs(steady_error)/f_true);
    fprintf('  Steady std: %.6f Hz\n', steady_std);
    fprintf('  Dominant interference freq: %.2f Hz\n', dominant_interference_freq);
    
    % Save individual result
    save(fullfile(results_dir, sprintf('multi_freq_%s.mat', cfg.name)), 'result');
    
    % Create detailed plot
    fig = figure('Position', [50 50 1400 1000]);
    
    % Subplot 1: Signal components
    subplot(4,1,1)
    plot(t, y, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
    hold on;
    plot(t, sig.y_primary, 'b-', 'LineWidth', 1);
    plot(t, sig.y_secondary, 'r-', 'LineWidth', 1);
    hold off;
    grid on;
    xlabel('Time [s]', 'FontSize', 11);
    ylabel('Amplitude', 'FontSize', 11);
    legend('Combined Signal', sprintf('Primary (%.1fHz)', cfg.f_primary), ...
           sprintf('Secondary (%.1fHz)', cfg.f_secondary), 'Location', 'best', 'FontSize', 10);
    title(sprintf('(a) Signal Components - %s', cfg.name), 'FontSize', 12, 'FontWeight', 'bold');
    xlim([0 10]);  % First 10 seconds
    
    % Subplot 2: Frequency estimation
    subplot(4,1,2)
    plot(t, est_freq_clean, 'b-', 'LineWidth', 1.5);
    hold on;
    plot([0 duration], [f_true f_true], 'r--', 'LineWidth', 2);
    plot([0 duration], [f_true+tolerance f_true+tolerance], 'k:', 'LineWidth', 1);
    plot([0 duration], [f_true-tolerance f_true-tolerance], 'k:', 'LineWidth', 1);
    if ~isnan(conv_time)
        plot([conv_time conv_time], ylim, 'g--', 'LineWidth', 1.5);
        text(conv_time+1, f_true*1.05, sprintf('Conv: %.2fs', conv_time), ...
             'FontSize', 10, 'Color', 'g', 'FontWeight', 'bold');
    end
    hold off;
    grid on;
    xlabel('Time [s]', 'FontSize', 11);
    ylabel('Frequency [Hz]', 'FontSize', 11);
    legend('Estimated', 'True Primary', '±2% Tolerance', 'Location', 'best', 'FontSize', 10);
    title('(b) Frequency Estimation', 'FontSize', 12, 'FontWeight', 'bold');
    xlim([0 duration]);
    
    % Subplot 3: RLS parameters
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
    title('(c) RLS Parameters', 'FontSize', 12, 'FontWeight', 'bold');
    xlim([0 duration]);
    
    % Subplot 4: Estimation error
    subplot(4,1,4)
    error_hz = est_freq_clean - f_true;
    plot(t, error_hz * 1000, 'Color', [0.8 0.4 0], 'LineWidth', 1.5);
    hold on;
    plot([0 duration], [0 0], 'k--', 'LineWidth', 1);
    hold off;
    grid on;
    xlabel('Time [s]', 'FontSize', 11);
    ylabel('Freq Error [mHz]', 'FontSize', 11);
    title(sprintf('(d) Frequency Error (Steady: %.3f ± %.3f mHz)', ...
                  steady_error*1000, steady_std*1000), ...
          'FontSize', 12, 'FontWeight', 'bold');
    xlim([0 duration]);
    
    sgtitle(sprintf('RLS Phase Estimation - Multi-Frequency Test: %s\n(Primary: %.1fHz, Secondary: %.1fHz, A_{sec}/A_{pri}=%.2f, α=%.2f)', ...
                    cfg.name, cfg.f_primary, cfg.f_secondary, cfg.A_secondary/cfg.A_primary, cfg.omega_alpha), ...
            'FontSize', 14, 'FontWeight', 'bold');
    
    saveas(fig, fullfile(results_dir, sprintf('multi_freq_%s.png', cfg.name)));
    close(fig);
end

% Save all results
save(fullfile(results_dir, 'multi_freq_all_results.mat'), 'results', 'configs');

% Create summary comparison
fprintf('\n=== Summary Comparison ===\n');
fprintf('%20s | %12s | %15s | %15s | %15s\n', ...
        'Config', 'Conv [s]', 'Error [Hz]', 'Std [Hz]', 'Interf Freq [Hz]');
fprintf('%s\n', repmat('-', 1, 85));
for i = 1:length(results)
    r = results{i};
    fprintf('%20s | %12.2f | %15.6f | %15.6f | %15.2f\n', ...
            r.config.name, r.conv_time, r.steady_error, r.steady_std, ...
            r.dominant_interference_freq);
end

fprintf('\n=== Multi-frequency test complete ===\n');
