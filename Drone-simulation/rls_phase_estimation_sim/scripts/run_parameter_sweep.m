% run_parameter_sweep: Comprehensive parameter sweep for RLS phase estimation
% Sweeps: alpha, buffer_size, update_interval, noise, initial_omega
clear; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);
results_dir = fullfile(script_dir,'..','results');
if ~exist(results_dir,'dir'), mkdir(results_dir); end

% Fixed parameters
fs = 100; duration = 60; f_true = 0.8; decay = 0.01;

% Sweep configurations
sweeps = struct();

% Sweep 1: omega_alpha (smoothing factor)
sweeps(1).name = 'alpha';
sweeps(1).values = [0.05, 0.1, 0.2, 0.3, 0.5];
sweeps(1).param = 'omega_alpha';
sweeps(1).fixed = struct('noise_std', 0.01, 'initial_omega', 0.6, 'phase_buffer_size', 300, 'phase_update_interval', 300);

% Sweep 2: phase_buffer_size
sweeps(2).name = 'buffer';
sweeps(2).values = [100, 200, 300, 400, 500];
sweeps(2).param = 'phase_buffer_size';
sweeps(2).fixed = struct('noise_std', 0.01, 'initial_omega', 0.6, 'omega_alpha', 0.2, 'phase_update_interval', 300);

% Sweep 3: phase_update_interval
sweeps(3).name = 'interval';
sweeps(3).values = [100, 200, 300, 400, 500];
sweeps(3).param = 'phase_update_interval';
sweeps(3).fixed = struct('noise_std', 0.01, 'initial_omega', 0.6, 'omega_alpha', 0.2, 'phase_buffer_size', 300);

% Sweep 4: noise_std
sweeps(4).name = 'noise';
sweeps(4).values = [0.0, 0.01, 0.02, 0.05, 0.1];
sweeps(4).param = 'noise_std';
sweeps(4).fixed = struct('initial_omega', 0.6, 'omega_alpha', 0.2, 'phase_buffer_size', 300, 'phase_update_interval', 300);

% Sweep 5: initial_omega
sweeps(5).name = 'init_omega';
sweeps(5).values = [0.4, 0.5, 0.6, 0.7, 0.9, 1.0];
sweeps(5).param = 'initial_omega';
sweeps(5).fixed = struct('noise_std', 0.01, 'omega_alpha', 0.2, 'phase_buffer_size', 300, 'phase_update_interval', 300);

% Run all sweeps
summary_results = cell(length(sweeps), 1);

for s = 1:length(sweeps)
    sweep = sweeps(s);
    fprintf('\n=== Sweep %d/%d: %s ===\n', s, length(sweeps), sweep.name);
    
    results = struct();
    results.sweep_name = sweep.name;
    results.param_name = sweep.param;
    results.values = sweep.values;
    results.conv_time = zeros(length(sweep.values), 1);
    results.steady_error = zeros(length(sweep.values), 1);
    results.steady_std = zeros(length(sweep.values), 1);
    results.final_freq = zeros(length(sweep.values), 1);
    
    for i = 1:length(sweep.values)
        val = sweep.values(i);
        fprintf('  Testing %s = %g\n', sweep.param, val);
        
        % Set up parameters
        fixed = sweep.fixed;
        if strcmp(sweep.param, 'noise_std')
            noise_std = val;
            initial_omega = fixed.initial_omega;
            omega_alpha = fixed.omega_alpha;
            phase_buffer_size = fixed.phase_buffer_size;
            phase_update_interval = fixed.phase_update_interval;
        elseif strcmp(sweep.param, 'initial_omega')
            noise_std = fixed.noise_std;
            initial_omega = val;
            omega_alpha = fixed.omega_alpha;
            phase_buffer_size = fixed.phase_buffer_size;
            phase_update_interval = fixed.phase_update_interval;
        else
            noise_std = fixed.noise_std;
            initial_omega = fixed.initial_omega;
            omega_alpha = strcmp(sweep.param, 'omega_alpha') && val || fixed.omega_alpha;
            phase_buffer_size = strcmp(sweep.param, 'phase_buffer_size') && val || fixed.phase_buffer_size;
            phase_update_interval = strcmp(sweep.param, 'phase_update_interval') && val || fixed.phase_update_interval;
            if strcmp(sweep.param, 'omega_alpha'), omega_alpha = val; end
            if strcmp(sweep.param, 'phase_buffer_size'), phase_buffer_size = val; phase_update_interval = val; end
            if strcmp(sweep.param, 'phase_update_interval'), phase_update_interval = val; end
        end
        
        % Generate signal
        outname = sprintf('sweep_%s_%.4g.csv', sweep.name, val);
        out = generate_damped_sine('fs', fs, 'duration', duration, 'f_true', f_true, ...
                                    'decay', decay, 'noise_std', noise_std, 'outname', outname);
        
        % Load data
        T = readtable(out.path);
        t = T.Time_s; y = T.Signal;
        
        % RLS parameters
        params = struct();
        params.lambda = 0.995;
        params.P0 = 1e3*eye(3);
        params.theta0 = zeros(3,1);
        params.initial_omega = initial_omega;
        params.phase_buffer_size = phase_buffer_size;
        params.phase_update_interval = phase_update_interval;
        params.phase_corr_enabled = true;
        params.phase_corr_thresh = 0.001;
        params.omega_alpha = omega_alpha;
        
        % Run estimation
        logs = rls_phase_estimator(y, t, params);
        
        % Compute metrics
        est_freq = logs.est_freq;
        est_freq(isnan(est_freq)) = initial_omega; % fill NaN with initial
        
        % Convergence time: when error < 2% of true value
        thresh = 0.02 * f_true;
        error = abs(est_freq - f_true);
        conv_idx = find(error < thresh, 1);
        if isempty(conv_idx)
            conv_time = NaN;
        else
            conv_time = t(conv_idx);
        end
        
        % Steady state: last 10 seconds
        steady_idx = t >= (duration - 10);
        steady_error = mean(est_freq(steady_idx) - f_true);
        steady_std = std(est_freq(steady_idx));
        final_freq = mean(est_freq(end-100:end));
        
        % Store results
        results.conv_time(i) = conv_time;
        results.steady_error(i) = steady_error;
        results.steady_std(i) = steady_std;
        results.final_freq(i) = final_freq;
        
        % Save individual result
        res_csv = sprintf('sweep_%s_%g.csv', sweep.name, val);
        resT = table(t, y, logs.ypred, logs.est_freq, logs.phase_corr, ...
                     'VariableNames', {'Time_s','Signal','Predicted','EstFreq_Hz','PhaseCorr'});
        writetable(resT, fullfile(results_dir, res_csv));
        
        % Plot
        meta.f_true = f_true; meta.decay = decay; meta.noise = noise_std;
        try
            plot_results(fullfile(results_dir, res_csv), out.path, meta);
        catch ME
            fprintf('  Warning: plotting failed: %s\n', ME.message);
        end
        
        fprintf('    Conv time: %.2f s, Steady error: %.4f Hz, Steady std: %.4f Hz\n', ...
                conv_time, steady_error, steady_std);
    end
    
    % Save sweep results
    summary_results{s} = results;
    save(fullfile(results_dir, sprintf('sweep_%s_summary.mat', sweep.name)), 'results');
    
    % Print summary table
    fprintf('\nSummary for %s sweep:\n', sweep.name);
    fprintf('%15s | %12s | %12s | %12s | %12s\n', sweep.param, 'ConvTime[s]', 'SteadyErr[Hz]', 'SteadyStd[Hz]', 'FinalFreq[Hz]');
    fprintf('%s\n', repmat('-', 1, 80));
    for i = 1:length(sweep.values)
        fprintf('%15g | %12.2f | %12.6f | %12.6f | %12.6f\n', ...
                sweep.values(i), results.conv_time(i), results.steady_error(i), ...
                results.steady_std(i), results.final_freq(i));
    end
    fprintf('\n');
end

% Save all results
save(fullfile(results_dir, 'all_sweep_results.mat'), 'summary_results', 'sweeps');
fprintf('Parameter sweep completed. Results saved to %s\n', results_dir);
