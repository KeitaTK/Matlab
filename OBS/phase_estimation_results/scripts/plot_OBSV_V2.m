%% OBSV_data_00000422.csv の全カラム可視化

clear; close all;

% === 設定 ===
dataDir = fullfile('..', 'data');
resultsDir = fullfile('..', 'results');
csvFileName = 'OBSV_data_00000422.csv';
dataPath = fullfile(dataDir, csvFileName);


try
    fprintf('ファイルを読み込み中...\n');
    data = readtable(dataPath);
    varNames = data.Properties.VariableNames;
    numVars = length(varNames);
    numPoints = height(data);
    
    % 時間軸（100Hz仮定）
    timeSec = (0:numPoints-1)' * 0.01;
    timeLabel = 'Time (s)';
    
    fprintf('データ読込完了: %d行, %d列\n', numPoints, numVars);
    fprintf('カラム名: %s\n', strjoin(varNames, ', '));
    
    % 出力ファイル名のベース（拡張子除去）
    [~, csvBaseName, ~] = fileparts(csvFileName);

    % 1カラムずつグラフ化
    for i = 1:numVars
        varName = varNames{i};
        if isnumeric(data.(varName))
            fig = figure('Visible','off','Position',[100,100,1200,400],'Name',varName);
            plot(timeSec, data.(varName), 'LineWidth', 1);
            grid on;
            xlabel(timeLabel);
            ylabel(varName);
            title(sprintf('変数: %s  (%d points)', varName, numPoints), 'Interpreter', 'none');
            % ファイル名を安全に
            safeVarName = regexprep(varName, '[^a-zA-Z0-9_]', '_');
            outputPath = fullfile(resultsDir, sprintf('%s_%s.png', csvBaseName, safeVarName));
            saveas(fig, outputPath);
            close(fig);
            fprintf('✓ %s を保存: %s\n', varName, outputPath);
        end
    end
    fprintf('\n全カラムのグラフ化が完了しました。\n');
catch ME
    fprintf('エラー: %s\n', ME.message);
end
