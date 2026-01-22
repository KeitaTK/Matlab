%% OBSV_data_00000404.csv の全カラム可視化
clear; close all;

% データファイルパス
dataPath = fullfile('..', 'data', 'OBSV_data_00000404.csv');

try
    fprintf('ファイルを読み込み中...\n');
    data = readtable(dataPath);
    varNames = data.Properties.VariableNames;
    numVars = length(varNames);
    numPoints = height(data);
    
    % 時間軸（100Hz仮定）
    timeSec = (0:numPoints-1)' * 0.01;
    timeLabel = 'Time (s)';
    
    % 1カラムずつグラフ化
    for i = 1:numVars
        varName = varNames{i};
        if isnumeric(data.(varName))
            fig = figure('Visible','off','Position',[100,100,1200,400],'Name',varName);
            plot(timeSec, data.(varName), 'LineWidth', 1);
            grid on;
            xlabel(timeLabel);
            ylabel(varName);
            title(sprintf('%s (%d points)', varName, numPoints));
            % ファイル名を安全に
            safeVarName = regexprep(varName, '[^a-zA-Z0-9_]', '_');
            outputPath = fullfile('..','results',sprintf('OBSV_00000404_%s.png',safeVarName));
            saveas(fig, outputPath);
            close(fig);
            fprintf('✓ %s を保存: %s\n', varName, outputPath);
        end
    end
    fprintf('全カラムのグラフ化が完了しました。\n');
catch ME
    fprintf('エラー: %s\n', ME.message);
end
