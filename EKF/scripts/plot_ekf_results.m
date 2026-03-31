function fig = plot_ekf_results(run_result, outPngPath)
% plot_ekf_results
% Generates summary plots for one EKF run.

t = run_result.signal.t;
y = run_result.signal.y;
y_hat = run_result.y_hat;
f_true = run_result.signal.true_freq_hz;
f_est = run_result.f_est_hz;
innov = run_result.innovation;

fig = figure('Position', [100, 100, 1300, 900]);

subplot(2,2,1);
plot(t, y, 'k', 'LineWidth', 1.0); hold on;
plot(t, y_hat, 'r', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]'); ylabel('Signal');
title('Signal and EKF Reconstruction');
legend('Measured', 'Estimated', 'Location', 'best');

subplot(2,2,2);
plot(t, f_true, 'k--', 'LineWidth', 1.5); hold on;
plot(t, f_est, 'b', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]'); ylabel('Frequency [Hz]');
title('Frequency Tracking');
legend('True', 'Estimated', 'Location', 'best');

subplot(2,2,3);
plot(t, innov, 'm', 'LineWidth', 1.0);
grid on;
xlabel('Time [s]'); ylabel('Innovation');
title('Innovation Time Series');

subplot(2,2,4);
freq_err = f_est - f_true;
plot(t, freq_err, 'b', 'LineWidth', 1.1); hold on;
yline(0, 'k--');
grid on;
xlabel('Time [s]'); ylabel('Frequency Error [Hz]');
title('Frequency Error');

sgtitle('Harmonic-Model EKF Summary', 'FontWeight', 'bold');

if nargin >= 2 && ~isempty(outPngPath)
    saveas(fig, outPngPath);
end
end
