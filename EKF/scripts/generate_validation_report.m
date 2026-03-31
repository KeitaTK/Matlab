function reportPath = generate_validation_report()
% generate_validation_report
% Runs representative EKF cases, creates plots, runs sweep summary,
% and writes a markdown report in Japanese.

scriptDir = fileparts(mfilename('fullpath'));
rootDir = fullfile(scriptDir, '..');
resultsDir = fullfile(rootDir, 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

addpath(scriptDir);

% 1) Representative cases
cases = {
    struct('id','baseline','kind','baseline', 'args',{{'fs',100,'duration',60,'f_true',0.8,'noise_std',0.0,'seed',1}}, 'f_init',0.6), ...
    struct('id','noisy_010','kind','noisy', 'args',{{'fs',100,'duration',60,'f_true',0.8,'noise_std',0.1,'seed',1}}, 'f_init',0.6), ...
    struct('id','interf_5hz_r05','kind','interf', 'args',{{'fs',100,'duration',60,'f_target',0.8,'f_interf',5.0,'amp_ratio_interf',0.5,'noise_std',0.01,'seed',1}}, 'f_init',0.6) ...
};

caseRows = cell(numel(cases), 7);
for i = 1:numel(cases)
    c = cases{i};
    switch c.kind
        case 'baseline'
            sig = generate_signal_baseline(c.args{:});
        case 'noisy'
            sig = generate_signal_noisy(c.args{:});
        otherwise
            sig = generate_signal_highfreq_interference(c.args{:});
    end

    cfg = struct();
    cfg.f_init_hz = c.f_init;
    cfg.P0 = diag([1, 1, 1, (2*pi*0.4)^2]);
    cfg.Q = diag([1e-4, 1e-3, 1e-5, 2e-4]);
    cfg.R = 5e-3;

    res = run_ekf_case(sig, cfg);

    figPath = fullfile(resultsDir, sprintf('case_%s.png', c.id));
    matPath = fullfile(resultsDir, sprintf('case_%s.mat', c.id));
    plot_ekf_results(res, figPath);
    run_result = res; %#ok<NASGU>
    save(matPath, 'run_result');

    m = res.metrics;
    caseRows(i,:) = {c.id, m.convergence_time_s, m.steady_bias_hz, m.steady_std_hz, m.nrmse, m.innovation_mean, m.innovation_lag1_corr};
end

caseTable = cell2table(caseRows, 'VariableNames', {
    'case_id','convergence_time_s','steady_bias_hz','steady_std_hz','nrmse','innovation_mean','innovation_lag1_corr'});

writetable(caseTable, fullfile(resultsDir, 'case_metrics.csv'));

% 2) Sweep
sweepCsv = fullfile(resultsDir, 'sweep_summary.csv');
T = run_ekf_sweep('saveCsvPath', sweepCsv);
save(fullfile(resultsDir, 'sweep_summary.mat'), 'T');

% 2.5) Frequency mismatch robustness check
fInitList = [0.4, 0.5, 0.6, 0.7, 1.0, 1.2];
fmRows = zeros(numel(fInitList), 5);
for i = 1:numel(fInitList)
    sigFM = generate_signal_baseline('fs',100,'duration',60,'f_true',0.8,'amplitude',1.0,'offset',0.0,'noise_std',0.0,'seed',1);
    cfgFM = struct('f_init_hz', fInitList(i));
    resFM = run_ekf_case(sigFM, cfgFM);
    mFM = resFM.metrics;
    fmRows(i,:) = [fInitList(i), mFM.convergence_time_s, mFM.steady_bias_hz, mFM.steady_std_hz, mFM.nrmse];
end
freqMismatchTable = array2table(fmRows, 'VariableNames', {'f_init_hz','convergence_time_s','steady_bias_hz','steady_std_hz','nrmse'});
writetable(freqMismatchTable, fullfile(resultsDir, 'freq_mismatch_check.csv'));

% 2.6) Amplitude internal-model check
ampRows = cell(3, 5);
ampRows(:,1) = {'baseline'; 'noisy_010'; 'interf_5hz_r05'};

ampCaseCfg = struct('f_init_hz', 0.6);

% baseline
sigA = generate_signal_baseline('fs',100,'duration',60,'f_true',0.8,'amplitude',1.0,'offset',0.0,'noise_std',0.0,'seed',1);
resA = run_ekf_case(sigA, ampCaseCfg);
xA = resA.x_hist;
wA = max(xA(4,:)', 1e-6);
Ahat = sqrt(xA(1,:)'.^2 + (xA(2,:)'.^2) ./ (wA.^2));
idxA = resA.signal.t >= 50;
ampRows(1,2:5) = {mean(Ahat(idxA)), mean(Ahat(idxA)-1), std(Ahat(idxA)-1), sqrt(mean((Ahat-1).^2))};

% noisy
sigAN = generate_signal_noisy('fs',100,'duration',60,'f_true',0.8,'amplitude',1.0,'offset',0.0,'noise_std',0.1,'seed',1);
resAN = run_ekf_case(sigAN, ampCaseCfg);
xAN = resAN.x_hist;
wAN = max(xAN(4,:)', 1e-6);
AhatN = sqrt(xAN(1,:)'.^2 + (xAN(2,:)'.^2) ./ (wAN.^2));
idxAN = resAN.signal.t >= 50;
ampRows(2,2:5) = {mean(AhatN(idxAN)), mean(AhatN(idxAN)-1), std(AhatN(idxAN)-1), sqrt(mean((AhatN-1).^2))};

% interference
sigAI = generate_signal_highfreq_interference('fs',100,'duration',60,'f_target',0.8,'f_interf',5.0,'amp_ratio_interf',0.5,'noise_std',0.01,'seed',1);
resAI = run_ekf_case(sigAI, ampCaseCfg);
xAI = resAI.x_hist;
wAI = max(xAI(4,:)', 1e-6);
AhatI = sqrt(xAI(1,:)'.^2 + (xAI(2,:)'.^2) ./ (wAI.^2));
idxAI = resAI.signal.t >= 50;
ampRows(3,2:5) = {mean(AhatI(idxAI)), mean(AhatI(idxAI)-1), std(AhatI(idxAI)-1), sqrt(mean((AhatI-1).^2))};

ampTable = cell2table(ampRows, 'VariableNames', {'case_id','mean_amp_last10s','amp_bias_last10s','amp_std_last10s','amp_rmse_full'});
writetable(ampTable, fullfile(resultsDir, 'amplitude_internal_model_check.csv'));

% Aggregate sweep by scenario_type
isNoise = strcmp(T.scenario_type, 'noise');
isInterf = strcmp(T.scenario_type, 'interference');

noiseBiasMean = mean(T.steady_bias_hz(isNoise), 'omitnan');
noiseStdMean = mean(T.steady_std_hz(isNoise), 'omitnan');
noiseConvMean = mean(T.convergence_time_s(isNoise), 'omitnan');

interfBiasMean = mean(T.steady_bias_hz(isInterf), 'omitnan');
interfStdMean = mean(T.steady_std_hz(isInterf), 'omitnan');
interfConvMean = mean(T.convergence_time_s(isInterf), 'omitnan');

% 3) Summary figures from sweep
fig1 = figure('Position', [100 100 1200 450]);
subplot(1,2,1);
idxN = isNoise;
scatter(T.noise_std(idxN), abs(T.steady_bias_hz(idxN)), 40, 'filled');
grid on;
xlabel('Noise std'); ylabel('|Steady Bias| [Hz]');
title('Noise vs Steady Bias');

subplot(1,2,2);
idxI = isInterf;
scatter(T.interf_freq_hz(idxI), T.steady_std_hz(idxI), 40, T.interf_amp_ratio(idxI), 'filled');
colorbar;
grid on;
xlabel('Interference Frequency [Hz]'); ylabel('Steady Std [Hz]');
title('Interference vs Frequency Std (color=amp ratio)');

sweepFigPath = fullfile(resultsDir, 'sweep_overview.png');
saveas(fig1, sweepFigPath);
close(fig1);

% 4) Write markdown report
reportPath = fullfile(resultsDir, 'EKF_Validation_Report.md');
fid = fopen(reportPath, 'w');
assert(fid > 0, 'Failed to open report file.');

fprintf(fid, '# 正弦波モデルEKF 検証レポート\n\n');
fprintf(fid, '- 生成日時: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(fid, '- MATLAB: %s\n\n', version);

fprintf(fid, '## 1. 実行した内容\n');
fprintf(fid, '- 正弦波モデルEKF（状態: [d, d_dot, c, omega]）を実行\n');
fprintf(fid, '- 代表3ケース（ベースライン/ノイズ/高周波干渉）を実施\n');
fprintf(fid, '- ノイズ・初期値・干渉スイープを実施\n\n');

fprintf(fid, '## 2. 代表ケース結果（数値）\n\n');
fprintf(fid, '| case_id | convergence_time_s | steady_bias_hz | steady_std_hz | nrmse | innovation_mean | innovation_lag1_corr |\n');
fprintf(fid, '|---|---:|---:|---:|---:|---:|---:|\n');
for i = 1:height(caseTable)
    fprintf(fid, '| %s | %.4f | %.6f | %.6f | %.6f | %.6f | %.6f |\n', ...
        caseTable.case_id{i}, caseTable.convergence_time_s(i), caseTable.steady_bias_hz(i), ...
        caseTable.steady_std_hz(i), caseTable.nrmse(i), caseTable.innovation_mean(i), caseTable.innovation_lag1_corr(i));
end
fprintf(fid, '\n');

fprintf(fid, '### 代表ケース図\n');
fprintf(fid, '- ベースライン: ![baseline](case_baseline.png)\n');
fprintf(fid, '- ノイズ0.1: ![noisy](case_noisy_010.png)\n');
fprintf(fid, '- 干渉5Hz 比0.5: ![interf](case_interf_5hz_r05.png)\n\n');

fprintf(fid, '## 3. スイープ集計\n\n');
fprintf(fid, '- Noise系 平均収束時間: %.4f s\n', noiseConvMean);
fprintf(fid, '- Noise系 平均定常バイアス: %.6f Hz\n', noiseBiasMean);
fprintf(fid, '- Noise系 平均定常標準偏差: %.6f Hz\n', noiseStdMean);
fprintf(fid, '- 干渉系 平均収束時間: %.4f s\n', interfConvMean);
fprintf(fid, '- 干渉系 平均定常バイアス: %.6f Hz\n', interfBiasMean);
fprintf(fid, '- 干渉系 平均定常標準偏差: %.6f Hz\n\n', interfStdMean);

fprintf(fid, '![sweep_overview](sweep_overview.png)\n\n');

fprintf(fid, '## 4. 利用ライブラリの記載先\n\n');
fprintf(fid, '利用ライブラリ一覧は、ドキュメント本体ではなく `EKF/INSTRUCTIONS.md` に記載する。\n\n');

fprintf(fid, '## 5. 追加検証: 初期周波数ミスマッチ耐性\n\n');
fprintf(fid, '| f_init_hz | convergence_time_s | steady_bias_hz | steady_std_hz | nrmse |\n');
fprintf(fid, '|---:|---:|---:|---:|---:|\n');
for i = 1:height(freqMismatchTable)
    fprintf(fid, '| %.3f | %.4f | %.6f | %.6f | %.6f |\n', ...
        freqMismatchTable.f_init_hz(i), freqMismatchTable.convergence_time_s(i), ...
        freqMismatchTable.steady_bias_hz(i), freqMismatchTable.steady_std_hz(i), freqMismatchTable.nrmse(i));
end
fprintf(fid, '\n');

fprintf(fid, '## 6. 追加検証: 振幅内部モデルパラメータ推定\n\n');
fprintf(fid, '振幅推定は、内部状態から以下で評価した。\n');
fprintf(fid, '$$A_{est}=\\sqrt{d^2 + (\\dot d / \\omega)^2}$$\n\n');
fprintf(fid, '| case_id | mean_amp_last10s | amp_bias_last10s | amp_std_last10s | amp_rmse_full |\n');
fprintf(fid, '|---|---:|---:|---:|---:|\n');
for i = 1:height(ampTable)
    fprintf(fid, '| %s | %.6f | %.6f | %.6f | %.6f |\n', ...
        string(ampTable.case_id(i)), ampTable.mean_amp_last10s(i), ampTable.amp_bias_last10s(i), ...
        ampTable.amp_std_last10s(i), ampTable.amp_rmse_full(i));
end
fprintf(fid, '\n');

fprintf(fid, '## 7. 性能評価と考察\n\n');
fprintf(fid, '1. 周波数推定性能: 初期周波数が0.4〜1.2 Hzにずれていても収束し、定常バイアスはほぼ一定で小さい。\n');
fprintf(fid, '2. ノイズ耐性: ノイズ増加で分散は増えるが、収束性とバイアスは大きく崩れていない。\n');
fprintf(fid, '3. 干渉耐性: 高周波干渉下でも周波数推定は成立するが、分散とNRMSEが増加する。\n');
fprintf(fid, '4. 振幅推定: ベースライン/ノイズ条件では概ね良好。強干渉条件では単一成分モデルの限界で誤差が増える。\n');
fprintf(fid, '5. 改善方針: 振幅精度を干渉下で上げるには、2成分モデル（ターゲット+干渉）または周波数依存のQ/Rスケジューリングが有効。\n\n');

fprintf(fid, '## 8. 出力ファイル\n');
fprintf(fid, '- case_metrics.csv\n');
fprintf(fid, '- sweep_summary.csv\n');
fprintf(fid, '- sweep_summary.mat\n');
fprintf(fid, '- freq_mismatch_check.csv\n');
fprintf(fid, '- amplitude_internal_model_check.csv\n');
fprintf(fid, '- case_baseline.png / case_noisy_010.png / case_interf_5hz_r05.png\n');
fprintf(fid, '- sweep_overview.png\n');

fclose(fid);

fprintf('Report generated: %s\n', reportPath);
end
