% run_decay_sweep: sweep over decay values and save annotated plots
clear; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);

decays = [0.01, 0.005, 0.002]; % smaller-than-current decay values
fs = 100; duration = 60; f_true = 0.8; noise_std = 0.01;

for d = decays
    decay = d;
    outname = sprintf('damped_080hz_60s_d%0.4f.csv', decay);
    out = generate_damped_sine('fs',fs,'duration',duration,'f_true',f_true,'decay',decay,'noise_std',noise_std,'outname',outname);
    datafile = out.path; T = readtable(datafile); t = T.Time_s; y = T.Signal;

    params = struct();
    params.lambda = 0.995;
    params.P0 = 1e3*eye(3);
    params.theta0 = zeros(3,1);
    params.initial_omega = 0.6;
    params.phase_buffer_size = 300;
    params.phase_update_interval = 300;
    params.phase_corr_enabled = true;
    params.phase_corr_thresh = 0.001;
    params.omega_alpha = 0.2;

    logs = rls_phase_estimator(y, t, params);

    results_dir = fullfile(script_dir,'..','results'); if ~exist(results_dir,'dir'), mkdir(results_dir); end
    base = sprintf('rls_sim_decay_d%0.4f.csv', decay);
    resT = table(t, y, logs.ypred, logs.est_freq, logs.phase_corr, 'VariableNames', {'Time_s','Signal','Predicted','EstFreq_Hz','PhaseCorr'});
    writetable(resT, fullfile(results_dir, base), 'FileType','text');

    meta.f_true = f_true; meta.decay = decay; meta.noise = noise_std;
    try
        plot_results(fullfile(results_dir, base), out.path, meta);
    catch ME
        fprintf('Warning: plotting failed for decay=%.4f: %s\n', decay, ME.message);
    end
end

fprintf('Decay sweep completed. Results in results/\n');