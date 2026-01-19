% run_sim_1hz: run simulation with f_true = 1.0 Hz
clear; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);

% parameters for 1 Hz test
fs = 100; duration = 60; f_true = 1.0; decay = 0.1; noise_std = 0.01; % default

% generate signal
out = generate_damped_sine('fs',fs,'duration',duration,'f_true',f_true,'decay',decay,'noise_std',noise_std,'outname','damped_100hz_60s.csv');

% load data
datafile = out.path; T = readtable(datafile); t = T.Time_s; y = T.Signal;

% RLS params
params = struct();
params.lambda = 0.995;
params.P0 = 1e3*eye(3);
params.theta0 = zeros(3,1);
params.initial_omega = 0.6; % initial guess (Hz)
params.phase_buffer_size = 300; % samples (~3 s)
params.phase_update_interval = 300; % update every 300 samples
params.phase_corr_enabled = true;
params.phase_corr_thresh = 0.001; % threshold
params.omega_alpha = 0.2; % smoothing

% run estimator
logs = rls_phase_estimator(y, t, params);

% save results
results_dir = fullfile(script_dir,'..','results'); if ~exist(results_dir,'dir'), mkdir(results_dir); end
save(fullfile(results_dir,'rls_sim_1hz.mat'),'logs','params','out');
res_table = table(t, y, logs.ypred, logs.est_freq, logs.phase_corr);
res_table.Properties.VariableNames = {'Time_s','Signal','Predicted','EstFreq_Hz','PhaseCorr'};
writetable(res_table, fullfile(results_dir,'rls_sim_1hz.csv'));

fprintf('1Hz simulation completed. Results saved to %s\n', results_dir);
meta.f_true = f_true; meta.decay = decay; meta.noise = noise_std; plot_results(fullfile(results_dir,'rls_sim_1hz.csv'), out.path, meta);
