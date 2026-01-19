function plot_results(result_csv, data_csv, meta)
% plot_results Plot simulation outputs and save figures
% Usage: plot_results(result_csv, data_csv, meta)
% meta (optional): struct with fields .f_true, .decay, .noise

T = readtable(result_csv);
D = readtable(data_csv);
script_dir = fileparts(mfilename('fullpath'));
results_dir = fullfile(script_dir,'..','results');
if ~exist(results_dir,'dir'), mkdir(results_dir); end

% metadata defaults
if nargin < 3, meta = struct(); end
if ~isfield(meta,'f_true'), meta.f_true = D.TrueFreq_Hz(1); end
if ~isfield(meta,'decay'), meta.decay = mean(D.TrueAmp) ./ max(D.TrueAmp); end
if ~isfield(meta,'noise'), meta.noise = NaN; end

t = T.Time_s;
figure('Visible','off','Position',[100 100 1000 800]);
subplot(3,1,1);
plot(t, T.Signal, 'k'); hold on; plot(t, T.Predicted, 'r');
legend('Signal','RLS Pred','Location','best'); xlabel('Time [s]'); ylabel('Signal');
title(sprintf('Signal vs Predicted (f=%.2f Hz, decay=%.3f)', meta.f_true, meta.decay));

subplot(3,1,2);
plot(t, T.EstFreq_Hz, '-b', 'LineWidth', 1.2); hold on; yline(meta.f_true, 'r--', 'True');
xlabel('Time [s]'); ylabel('Est F (Hz)'); title('Estimated Frequency');

subplot(3,1,3);
plot(t, T.PhaseCorr, '-m'); xlabel('Time [s]'); ylabel('Phase Corr (rad)'); title('Phase Correction');

% annotate with metadata text box
annotation_str = sprintf('f=%.3f Hz\ndecay=%.4f\nnoise=%.4f', meta.f_true, meta.decay, meta.noise);
annotation('textbox',[0.75 0.02 0.23 0.15],'String',annotation_str,'EdgeColor','none','FontSize',10);

% Save with sequential filename that includes frequency and decay
prefix = sprintf('rls_sim_summary_f%.2f_d%.4f', meta.f_true, meta.decay);
% sanitize filename (replace dot with p to avoid confusion)
prefix_safe = strrep(prefix, '.', 'p');
files = dir(fullfile(results_dir, [prefix_safe, '_*.png']));
next_num = numel(files) + 1;
outname = fullfile(results_dir, sprintf('%s_%03d.png', prefix_safe, next_num));
saveas(gcf, outname);
fprintf('Plots saved to %s\n', outname);
end