function metrics = evaluate_ekf_metrics(t, y, y_hat, f_true_hz, f_est_hz, innovation)
% evaluate_ekf_metrics
% Computes core metrics for EKF validation.

if nargin < 6
    innovation = y - y_hat;
end

N = numel(t);
lastMask = t >= (t(end) - 10);

freq_err = f_est_hz - f_true_hz;

% Convergence time to 2% band
band = 0.02 * max(mean(f_true_hz), eps);
idx = find(abs(freq_err) < band, 1, 'first');
if isempty(idx)
    conv_time = NaN;
else
    conv_time = t(idx);
end

steady_bias = mean(freq_err(lastMask));
steady_std = std(freq_err(lastMask));

rmse = sqrt(mean((y - y_hat).^2));
y_range = max(y) - min(y);
if y_range < eps
    nrmse = NaN;
else
    nrmse = rmse / y_range;
end

innov_mean = mean(innovation);
if N >= 2
    rho1 = corr(innovation(2:end), innovation(1:end-1), 'Rows', 'complete');
else
    rho1 = NaN;
end

metrics = struct();
metrics.convergence_time_s = conv_time;
metrics.steady_bias_hz = steady_bias;
metrics.steady_std_hz = steady_std;
metrics.nrmse = nrmse;
metrics.innovation_mean = innov_mean;
metrics.innovation_lag1_corr = rho1;
end
