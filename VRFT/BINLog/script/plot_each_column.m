% PSCE_data_00000004.csv の各カラムを1枚ずつ図にして保存するスクリプト
% 出力: 各カラムごとにPNG画像を../resultディレクトリに保存

clear; close all;

% === 設定 ===
inputCsv = fullfile('..','CSV', 'PSCE_data_00000004.csv');
outputDir = fullfile('..','result');

% CSVデータをテーブルとして読み込み
opts = detectImportOptions(inputCsv, 'NumHeaderLines', 0);
data = readtable(inputCsv, opts);
varNames = data.Properties.VariableNames;
numVars = length(varNames);
numPoints = height(data);

% 時間軸（TimeUSを秒換算）
if ismember('TimeUS', varNames)
    timeSec = (data.TimeUS - data.TimeUS(1)) * 1e-6;
    timeLabel = 'Time (s)';
else
    timeSec = (0:numPoints-1)';
    timeLabel = 'Index';
end

% 出力ディレクトリ作成
if ~isfolder(outputDir)
    mkdir(outputDir);
end

% 各カラムを1枚ずつ図にして保存
for i = 1:numVars
    varName = varNames{i};
    if strcmp(varName, 'TimeUS')
        continue;
    end
    fig = figure('Visible','off','Position',[100,100,1200,400],'Name',varName);
    plot(timeSec, data.(varName), 'LineWidth', 1.5);
    grid on;
    xlabel(timeLabel);
    ylabel(varName);
    title(sprintf('%s vs %s', varName, timeLabel), 'Interpreter', 'none');
    % ファイル名を安全に
    safeVarName = regexprep(varName, '[^a-zA-Z0-9_]', '_');
    outputPath = fullfile(outputDir, sprintf('%s.png', safeVarName));
    saveas(fig, outputPath);
    close(fig);
    fprintf('✓ %s を保存\n', varName);
end

fprintf('全カラムの図を保存しました。\n保存先: %s\n', outputDir);
