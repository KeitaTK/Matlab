%% OBSV_data.csv のデータをグラフ化するスクリプト
% データファイルのパス
dataPath = fullfile('..', 'data', 'OBSV_data.csv');
resultsPath = fullfile('..', 'results');

% 結果保存用のディレクトリが存在しない場合は作成
if ~exist(resultsPath, 'dir')
    mkdir(resultsPath);
end

fprintf('================================================================================\n');
fprintf('OBSV_data.csv データ可視化\n');
fprintf('================================================================================\n\n');

try
    %% データ読み込み
    fprintf('データ読み込み中...\n');
    data = readtable(dataPath);
    fprintf('✓ データ読み込み完了 (%d 行)\n', height(data));
    fprintf('  時間軸: 100Hzサンプリング (10ms周期)\n');
    fprintf('  総時間: %.2f 秒\n\n', (height(data)-1) * 0.01);
    
    % 変数名を取得
    varNames = data.Properties.VariableNames;
    
    % 時間軸: 100Hzサンプリング仮定
    % TimeUSは信頼できないため、行番号ベースで計算
    nSamples = height(data);
    samplingFreq = 100;  % Hz
    samplingPeriod = 1 / samplingFreq;  % 秒 (0.01 s = 10 ms)
    timeSec = (0:nSamples-1)' * samplingPeriod;
    data.TimeSec = timeSec;
    
    %% グラフ1: 位置関連データ (PLX, PLY, PLZ)
    fprintf('グラフ1を作成中: 位置関連データ (PLX, PLY, PLZ)\n');
    fig1 = figure('Name', 'Position Data', 'Position', [100, 100, 1200, 800]);
    
    positionVars = {'PLX', 'PLY', 'PLZ'};
    availablePos = intersect(positionVars, varNames);
    
    if ~isempty(availablePos)
        for i = 1:length(availablePos)
            subplot(length(availablePos), 1, i);
            plot(data.TimeSec, data.(availablePos{i}), 'LineWidth', 1.5);
            ylabel(availablePos{i});
            xlabel('Time [s]');
            title(sprintf('%s vs Time', availablePos{i}));
            grid on;
        end
        saveas(fig1, fullfile(resultsPath, 'position_data.png'));
        saveas(fig1, fullfile(resultsPath, 'position_data.fig'));
        fprintf('✓ 位置データグラフ保存完了\n');
    else
        fprintf('! 位置データなし\n');
        close(fig1);
    end
    
    %% グラフ2: A系列データ (AX, AY)
    fprintf('グラフ2を作成中: A系列データ (AX, AY)\n');
    fig2 = figure('Name', 'A Series Data', 'Position', [120, 120, 1200, 800]);
    
    aVars = {'AX', 'AY'};
    availableA = intersect(aVars, varNames);
    
    if ~isempty(availableA)
        for i = 1:length(availableA)
            subplot(length(availableA), 1, i);
            plot(data.TimeSec, data.(availableA{i}), 'LineWidth', 1.5);
            ylabel(availableA{i});
            xlabel('Time [s]');
            title(sprintf('%s vs Time', availableA{i}));
            grid on;
        end
        saveas(fig2, fullfile(resultsPath, 'a_series_data.png'));
        saveas(fig2, fullfile(resultsPath, 'a_series_data.fig'));
        fprintf('✓ A系列データグラフ保存完了\n');
    else
        fprintf('! A系列データなし\n');
        close(fig2);
    end
    
    %% グラフ3: B系列データ (BX, BY)
    fprintf('グラフ3を作成中: B系列データ (BX, BY)\n');
    fig3 = figure('Name', 'B Series Data', 'Position', [140, 140, 1200, 800]);
    
    bVars = {'BX', 'BY'};
    availableB = intersect(bVars, varNames);
    
    if ~isempty(availableB)
        for i = 1:length(availableB)
            subplot(length(availableB), 1, i);
            plot(data.TimeSec, data.(availableB{i}), 'LineWidth', 1.5);
            ylabel(availableB{i});
            xlabel('Time [s]');
            title(sprintf('%s vs Time', availableB{i}));
            grid on;
        end
        saveas(fig3, fullfile(resultsPath, 'b_series_data.png'));
        saveas(fig3, fullfile(resultsPath, 'b_series_data.fig'));
        fprintf('✓ B系列データグラフ保存完了\n');
    else
        fprintf('! B系列データなし\n');
        close(fig3);
    end
    
    %% グラフ4: C系列データ (CX, CY)
    fprintf('グラフ4を作成中: C系列データ (CX, CY)\n');
    fig4 = figure('Name', 'C Series Data', 'Position', [160, 160, 1200, 800]);
    
    cVars = {'CX', 'CY'};
    availableC = intersect(cVars, varNames);
    
    if ~isempty(availableC)
        for i = 1:length(availableC)
            subplot(length(availableC), 1, i);
            plot(data.TimeSec, data.(availableC{i}), 'LineWidth', 1.5);
            ylabel(availableC{i});
            xlabel('Time [s]');
            title(sprintf('%s vs Time', availableC{i}));
            grid on;
        end
        saveas(fig4, fullfile(resultsPath, 'c_series_data.png'));
        saveas(fig4, fullfile(resultsPath, 'c_series_data.fig'));
        fprintf('✓ C系列データグラフ保存完了\n');
    else
        fprintf('! C系列データなし\n');
        close(fig4);
    end
    
    %% グラフ5: PR系列データ (PRX, PRY, PRZ)
    fprintf('グラフ5を作成中: PR系列データ (PRX, PRY, PRZ)\n');
    fig5 = figure('Name', 'PR Series Data', 'Position', [180, 180, 1200, 800]);
    
    prVars = {'PRX', 'PRY', 'PRZ'};
    availablePR = intersect(prVars, varNames);
    
    if ~isempty(availablePR)
        for i = 1:length(availablePR)
            subplot(length(availablePR), 1, i);
            plot(data.TimeSec, data.(availablePR{i}), 'LineWidth', 1.5);
            ylabel(availablePR{i});
            xlabel('Time [s]');
            title(sprintf('%s vs Time', availablePR{i}));
            grid on;
        end
        saveas(fig5, fullfile(resultsPath, 'pr_series_data.png'));
        saveas(fig5, fullfile(resultsPath, 'pr_series_data.fig'));
        fprintf('✓ PR系列データグラフ保存完了\n');
    else
        fprintf('! PR系列データなし\n');
        close(fig5);
    end
    
    %% グラフ6: 推定結果 (ERR, EST_FREQ, CORR)
    fprintf('グラフ6を作成中: 推定結果 (ERR, EST_FREQ, CORR)\n');
    fig6 = figure('Name', 'Estimation Results', 'Position', [200, 200, 1200, 900]);
    
    estVars = {'ERR', 'EST_FREQ', 'CORR'};
    availableEst = intersect(estVars, varNames);
    
    if ~isempty(availableEst)
        for i = 1:length(availableEst)
            subplot(length(availableEst), 1, i);
            plot(data.TimeSec, data.(availableEst{i}), 'LineWidth', 1.5);
            ylabel(availableEst{i});
            xlabel('Time [s]');
            title(sprintf('%s vs Time', availableEst{i}));
            grid on;
        end
        saveas(fig6, fullfile(resultsPath, 'estimation_results.png'));
        saveas(fig6, fullfile(resultsPath, 'estimation_results.fig'));
        fprintf('✓ 推定結果グラフ保存完了\n');
    else
        fprintf('! 推定結果データなし\n');
        close(fig6);
    end
    
    %% グラフ7: 全データの概観
    fprintf('グラフ7を作成中: 全データ概観\n');
    fig7 = figure('Name', 'All Data Overview', 'Position', [220, 220, 1400, 1000]);
    
    % TimeUS以外のすべてのカラムをプロット
    plotVars = setdiff(varNames, {'TimeUS', 'TimeSec'});
    nVars = length(plotVars);
    nCols = 3;
    nRows = ceil(nVars / nCols);
    
    for i = 1:nVars
        subplot(nRows, nCols, i);
        plot(data.TimeSec, data.(plotVars{i}), 'LineWidth', 1);
        ylabel(plotVars{i});
        xlabel('Time [s]');
        title(plotVars{i});
        grid on;
    end
    
    saveas(fig7, fullfile(resultsPath, 'all_data_overview.png'));
    saveas(fig7, fullfile(resultsPath, 'all_data_overview.fig'));
    fprintf('✓ 全データ概観グラフ保存完了\n');
    
    fprintf('\n================================================================================\n');
    fprintf('すべてのグラフ作成完了\n');
    fprintf('保存先: %s\n', resultsPath);
    fprintf('================================================================================\n');
    
catch ME
    fprintf('\n✗ エラー発生: %s\n', ME.message);
    fprintf('スタックトレース:\n');
    for i = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
end
