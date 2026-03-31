function summaryTable = run_ekf_sweep(varargin)
% run_ekf_sweep
% Runs baseline/noisy/interference EKF tests and returns summary table.

p = inputParser;
addParameter(p, 'fs', 100);
addParameter(p, 'duration', 60);
addParameter(p, 'f_true', 0.8);
addParameter(p, 'f_init_list', [0.4, 0.6, 1.0]);
addParameter(p, 'noise_list', [0.0, 0.05, 0.1]);
addParameter(p, 'interf_freq_list', [3, 5, 8]);
addParameter(p, 'interf_ratio_list', [0.3, 0.5]);
addParameter(p, 'seed', 1);
addParameter(p, 'saveCsvPath', '');
parse(p, varargin{:});

cfg = p.Results;
rows = {};

% Baseline and noise sweep
for i = 1:numel(cfg.noise_list)
    noiseStd = cfg.noise_list(i);
    sig = generate_signal_noisy( ...
        'fs', cfg.fs, 'duration', cfg.duration, 'f_true', cfg.f_true, ...
        'noise_std', noiseStd, 'seed', cfg.seed);

    for j = 1:numel(cfg.f_init_list)
        f0 = cfg.f_init_list(j);
        ekfCfg = struct('f_init_hz', f0);
        res = run_ekf_case(sig, ekfCfg);
        m = res.metrics;

        rows(end+1,:) = {'noise', noiseStd, NaN, NaN, f0, m.convergence_time_s, ...
            m.steady_bias_hz, m.steady_std_hz, m.nrmse, m.innovation_mean, m.innovation_lag1_corr}; %#ok<AGROW>
    end
end

% High-frequency interference sweep
for i = 1:numel(cfg.interf_freq_list)
    fi = cfg.interf_freq_list(i);
    for j = 1:numel(cfg.interf_ratio_list)
        ratio = cfg.interf_ratio_list(j);

        sig = generate_signal_highfreq_interference( ...
            'fs', cfg.fs, 'duration', cfg.duration, ...
            'f_target', cfg.f_true, 'f_interf', fi, ...
            'amp_ratio_interf', ratio, 'seed', cfg.seed);

        ekfCfg = struct('f_init_hz', 0.6);
        res = run_ekf_case(sig, ekfCfg);
        m = res.metrics;

        rows(end+1,:) = {'interference', NaN, fi, ratio, 0.6, m.convergence_time_s, ...
            m.steady_bias_hz, m.steady_std_hz, m.nrmse, m.innovation_mean, m.innovation_lag1_corr}; %#ok<AGROW>
    end
end

summaryTable = cell2table(rows, 'VariableNames', {
    'scenario_type','noise_std','interf_freq_hz','interf_amp_ratio','f_init_hz', ...
    'convergence_time_s','steady_bias_hz','steady_std_hz','nrmse','innovation_mean','innovation_lag1_corr'});

if ~isempty(cfg.saveCsvPath)
    writetable(summaryTable, cfg.saveCsvPath);
end
end
