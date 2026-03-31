function out = run_ekf_case(signal, cfg)
% run_ekf_case
% Runs harmonic-model EKF on a given signal struct.
%
% Required signal fields: t, y, true_freq_hz

if nargin < 2
    cfg = struct();
end
if ~isfield(cfg, 'f_init_hz'), cfg.f_init_hz = 0.6; end
if ~isfield(cfg, 'x0'), cfg.x0 = [0; 0; 0; 2*pi*cfg.f_init_hz]; end
if ~isfield(cfg, 'P0'), cfg.P0 = diag([1, 1, 1, (2*pi*0.3)^2]); end
if ~isfield(cfg, 'Q'), cfg.Q = diag([1e-4, 1e-3, 1e-5, 1e-4]); end
if ~isfield(cfg, 'R'), cfg.R = 1e-2; end
if ~isfield(cfg, 'omega_min'), cfg.omega_min = 0.2; end
if ~isfield(cfg, 'omega_max'), cfg.omega_max = 15.0; end
if ~isfield(cfg, 'save_mat_path'), cfg.save_mat_path = ''; end

N = numel(signal.t);

x_hist = zeros(4, N);
P_hist = zeros(4, 4, N);
y_hat = zeros(N, 1);
innov = zeros(N, 1);
S_hist = zeros(N, 1);

xk = cfg.x0(:);
Pk = cfg.P0;

opts = struct('omega_min', cfg.omega_min, 'omega_max', cfg.omega_max, 'symmetrizeP', true);

for k = 1:N
    if k == 1
        dt = signal.t(2) - signal.t(1);
    else
        dt = signal.t(k) - signal.t(k-1);
    end

    [xk, Pk, dbg] = ekf_harmonic_step(xk, Pk, signal.y(k), dt, cfg.Q, cfg.R, opts);

    x_hist(:, k) = xk;
    P_hist(:, :, k) = Pk;
    y_hat(k) = dbg.y_pred;
    innov(k) = dbg.innov;
    S_hist(k) = dbg.S;
end

f_est_hz = x_hist(4, :)' / (2 * pi);

out = struct();
out.signal = signal;
out.cfg = cfg;
out.x_hist = x_hist;
out.P_hist = P_hist;
out.y_hat = y_hat;
out.innovation = innov;
out.S = S_hist;
out.f_est_hz = f_est_hz;
out.metrics = evaluate_ekf_metrics(signal.t, signal.y, y_hat, signal.true_freq_hz, f_est_hz, innov);

if ~isempty(cfg.save_mat_path)
    run_result = out; %#ok<NASGU>
    save(cfg.save_mat_path, 'run_result');
end
end
