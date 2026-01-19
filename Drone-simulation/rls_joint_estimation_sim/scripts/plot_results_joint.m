function plot_results_joint(results, t, y, true_params, ...
    convergence_time, steady_error_mean, steady_error_std, filename_base)
% PLOT_RESULTS_JOINT ジョイント推定結果をプロット
%
% 入力:
%   results              - rls_joint_estimator の出力構造体
%   t                    - 時刻ベクトル
%   y                    - 観測信号
%   true_params          - 真値パラメータ構造体
%   convergence_time     - 収束時間 [s]
%   steady_error_mean    - 定常誤差平均 [Hz]
%   steady_error_std     - 定常誤差標準偏差 [Hz]
%   filename_base        - 保存ファイル名（拡張子なし）

% 周波数推定値
freq_est = results.omega_hist / (2*pi);

% 真値
A_true = true_params.A;
B_true = true_params.B;
C_true = true_params.C;
f_true = true_params.freq;

% ========== プロット ==========
fig = figure('Position', [100, 100, 1200, 900]);

% サブプロット1: 信号と予測
subplot(4, 1, 1);
plot(t, y, 'b-', 'LineWidth', 0.5, 'DisplayName', '観測信号');
hold on;
plot(t, results.y_pred_hist, 'r--', 'LineWidth', 1.5, 'DisplayName', '予測信号');
xlabel('時刻 [s]');
ylabel('振幅');
title('観測信号 vs 予測信号');
legend('Location', 'best');
grid on;

% サブプロット2: 周波数推定
subplot(4, 1, 2);
plot(t, freq_est, 'b-', 'LineWidth', 1.5, 'DisplayName', '推定周波数');
hold on;
yline(f_true, 'r--', 'LineWidth', 2, 'DisplayName', '真値');
if ~isnan(convergence_time)
    xline(convergence_time, 'g--', 'LineWidth', 1.5, 'DisplayName', sprintf('収束 (%.2f s)', convergence_time));
end
xlabel('時刻 [s]');
ylabel('周波数 [Hz]');
title('周波数推定');
legend('Location', 'best');
grid on;

% サブプロット3: パラメータ A, B, C
subplot(4, 1, 3);
plot(t, results.theta_hist(:, 1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'A 推定');
hold on;
plot(t, results.theta_hist(:, 2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'B 推定');
plot(t, results.theta_hist(:, 3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'C 推定');
yline(A_true, 'r--', 'LineWidth', 1, 'HandleVisibility', 'off');
yline(B_true, 'g--', 'LineWidth', 1, 'HandleVisibility', 'off');
yline(C_true, 'b--', 'LineWidth', 1, 'HandleVisibility', 'off');
xlabel('時刻 [s]');
ylabel('パラメータ値');
title('A, B, C 推定値');
legend('Location', 'best');
grid on;

% サブプロット4: 推定誤差と共分散トレース
subplot(4, 1, 4);
yyaxis left;
plot(t, results.error_hist, 'b-', 'LineWidth', 0.5);
ylabel('誤差');
yyaxis right;
plot(t, results.P_hist, 'r-', 'LineWidth', 1.5);
ylabel('共分散トレース');
xlabel('時刻 [s]');
title('推定誤差 & 共分散');
grid on;

% 全体タイトル（メタデータ）
sgtitle(sprintf(['Joint RLS+Gradient | f_{true}=%.2f Hz, decay=%.3f, ' ...
    '収束時間=%.2f s, 定常誤差=%.4f±%.4f Hz'], ...
    f_true, true_params.decay, convergence_time, steady_error_mean, steady_error_std), ...
    'FontSize', 12, 'FontWeight', 'bold');

% 保存
saveas(fig, [filename_base '.png']);
fprintf('プロット保存: %s.png\n', filename_base);

end
