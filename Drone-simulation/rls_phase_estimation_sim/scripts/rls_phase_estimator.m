function logs = rls_phase_estimator(y, t, params)
% rls_phase_estimator  Online RLS-based phase/frequency estimator
%   logs = rls_phase_estimator(y, t, params)
% Inputs:
%   y: signal vector (Nx1)
%   t: time vector (Nx1)
%   params: struct with fields (optional defaults shown)
%     .lambda = 0.995
%     .P0 = 1e3*eye(3)
%     .theta0 = zeros(3,1)
%     .initial_omega = 0.6; % Hz (user requested initial guess)
%     .phase_buffer_size = 300; % samples
%     .phase_update_interval = 100; % samples
%     .phase_corr_enabled = true;
%     .phase_corr_thresh = 10.0; % rad
%     .min_cov = 1e-6; .max_cov = 1e8;
% Outputs in logs struct: theta_hist, ypred, est_freq, phase_corr_hist, P_hist

if nargin < 3, params = struct(); end
if ~isfield(params,'lambda'), params.lambda = 0.995; end
if ~isfield(params,'P0'), params.P0 = 1e3*eye(3); end
if ~isfield(params,'theta0'), params.theta0 = zeros(3,1); end
if ~isfield(params,'initial_omega'), params.initial_omega = 0.6; end
if ~isfield(params,'phase_buffer_size'), params.phase_buffer_size = 300; end
if ~isfield(params,'phase_update_interval'), params.phase_update_interval = 300; end
if ~isfield(params,'phase_corr_enabled'), params.phase_corr_enabled = true; end
if ~isfield(params,'phase_corr_thresh'), params.phase_corr_thresh = 10.0; end
if ~isfield(params,'omega_alpha'), params.omega_alpha = 0.2; end
if ~isfield(params,'min_cov'), params.min_cov = 1e-6; end
if ~isfield(params,'max_cov'), params.max_cov = 1e8; end

N = length(y);
dt = mean(diff(t));

% state
theta = params.theta0(:);
P = params.P0;
phase_correction = 0.0;
phase_buf = zeros(params.phase_buffer_size,1);
phase_buf_idx = 1;
phase_buf_count = 0;
phase_initialized = false;

% logs
theta_hist = zeros(3,N);
ypred = zeros(N,1);
est_freq = nan(N,1);
phase_corr_hist = zeros(N,1);

omega_param = params.initial_omega; % Hz - used to build phase
omega_rad_param = 2*pi*omega_param;

for i = 1:N
    ti = t(i);
    % compute model phase for basis (used for x) and also compute phase from RLS coeffs
    phase_basis = omega_rad_param * ti - phase_correction;

    % RLS input vector uses the basis phase
    x = [sin(phase_basis); cos(phase_basis); 1.0];

    % compute phase from RLS coefficients (complex projection angle)
    Z = theta(1) + 1i * theta(2);  % A + i*B
    ph = angle(Z);

    % unwrap estimated-phase relative to previous
    if ~phase_initialized
        prev_phase = ph;
        phase_initialized = true;
    else
        d = ph - prev_phase;
        if d > pi, ph = ph - 2*pi; end
        if d < -pi, ph = ph + 2*pi; end
        prev_phase = ph;
    end

    % push estimated phase to buffer
    phase_buf(phase_buf_idx) = ph;
    phase_buf_idx = phase_buf_idx + 1;
    if phase_buf_idx > params.phase_buffer_size, phase_buf_idx = 1; end
    phase_buf_count = min(phase_buf_count + 1, params.phase_buffer_size);

    % RLS update
    lambda = params.lambda;
    Px = P * x;
    denom = lambda + x' * Px;
    if abs(denom) < 1e-12
        denom = 1e-12;
    end
    K = Px / denom;
    y_n = y(i);
    y_pred = x' * theta;
    e = y_n - y_pred;
    theta = theta + K * e;
    P = (P - K * (x') * P) / lambda;
    % clamp P
    P = max(min(P, params.max_cov), params.min_cov);

    theta_hist(:,i) = theta;
    ypred(i) = y_pred;

    % phase correction update periodically when buffer full
    if params.phase_corr_enabled && (mod(i, params.phase_update_interval) == 0) && (phase_buf_count == params.phase_buffer_size)
        % reorder buffer in time order
        idx_start = phase_buf_idx;
        ordered = zeros(phase_buf_count,1);
        for k = 1:phase_buf_count
            idx = mod(idx_start - 1 + k - 1, params.phase_buffer_size) + 1;
            ordered(k) = phase_buf(idx);
        end
        % unwrap the ordered buffer
        ordered = unwrap(ordered);
        % linear fit to compute slope vs sample index (slope in rad/sample)
        xvec = (0:phase_buf_count-1)';
        pcoef = polyfit(xvec, ordered, 1);
        slope = pcoef(1); % rad per sample (this is slope of angle(Z))
        % convert to rad/s (delta omega in rad/s)
        delta_omega_rad = slope / dt; % rad/s
        % clamp delta to avoid instability
        max_step = 1.0; % rad/s (reduced)
        delta_omega_rad = max(min(delta_omega_rad, max_step), -max_step);
        % smoothing update
        alpha = params.omega_alpha;
        omega_rad_param = omega_rad_param + alpha * delta_omega_rad;
        est_freq_i = omega_rad_param / (2*pi); % Hz
        est_freq(i) = est_freq_i;
        % optionally update phase correction to keep continuity (small adjustment)
        % keep phase_correction unchanged to avoid abrupt jumps
        % debug print (small cadence)
        if mod(i, params.phase_update_interval) == 0
            fprintf('PhaseUpdate t=%.2fs slope=%.6f rad/sample delta_omega=%.4f rad/s alpha=%.2f estf=%.4f Hz\n', ti, slope, delta_omega_rad, alpha, est_freq_i);
        end
    else
        % interpolate est_freq
        if i>1, est_freq(i) = est_freq(i-1); else est_freq(i) = omega_param; end
    end
    phase_corr_hist(i) = phase_correction;
end

% pack logs
logs.theta_hist = theta_hist;
logs.ypred = ypred;
logs.est_freq = est_freq;
logs.phase_corr = phase_corr_hist;
logs.P_final = P;
logs.t = t;

end