%% run_demo_harmonic_ekf
% Quick demo for harmonic-model EKF.

clc; clear; close all;

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fullfile(scriptDir, '..');
resultDir = fullfile(projectRoot, 'results');
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end

% 1) Generate a high-frequency interference case
sig = generate_signal_highfreq_interference( ...
    'fs', 100, ...
    'duration', 60, ...
    'f_target', 0.8, ...
    'f_interf', 5.0, ...
    'amp_ratio_interf', 0.3, ...
    'noise_std', 0.02, ...
    'seed', 1);

% 2) Run EKF
cfg = struct();
cfg.f_init_hz = 0.6;
cfg.P0 = diag([1, 1, 1, (2*pi*0.4)^2]);
cfg.Q = diag([1e-4, 1e-3, 1e-5, 2e-4]);
cfg.R = 5e-3;

res = run_ekf_case(sig, cfg);

% 3) Plot and save
pngPath = fullfile(resultDir, 'demo_harmonic_ekf.png');
plot_ekf_results(res, pngPath);

% 4) Print metrics
disp('--- EKF metrics ---');
disp(res.metrics);

% 5) Save result mat
matPath = fullfile(resultDir, 'demo_harmonic_ekf.mat');
run_result = res; %#ok<NASGU>
save(matPath, 'run_result');

fprintf('Saved figure: %s\n', pngPath);
fprintf('Saved MAT: %s\n', matPath);
