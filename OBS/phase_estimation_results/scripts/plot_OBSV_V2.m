%% OBSV_data の全カラム可視化（複数ファイル対応）

clear; close all;

% === 設定 ===
dataDir = fullfile('..', 'data');
resultsDir = fullfile('..', 'results');

% 処理対象のCSVファイルを指定 （このセルアレイを編集してください）
csvFileNames = {
    % 'OBSV_data_00000420.csv'
    'OBSV_data_00000422.csv'
};

% 指定されたファイルの存在確認
csvFiles = [];
for i = 1:length(csvFileNames)
    filePath = fullfile(dataDir, csvFileNames{i});
    if isfile(filePath)
        csvFiles = [csvFiles; struct('name', csvFileNames{i}, 'folder', dataDir)];
    else
        fprintf('警告: ファイルが見つかりません: %s\n', csvFileNames{i});
    end
end

if isempty(csvFiles)
    fprintf('エラー: 処理対象のCSVファイルが見つかりません\n');
    return;
end

fprintf('合計 %d 個のCSVファイルを処理します\n\n', length(csvFiles));

% 各CSVファイルを処理
for fileIdx = 1:length(csvFiles)
    csvFileName = csvFiles(fileIdx).name;
    dataPath = fullfile(dataDir, csvFileName);
    
    try
        fprintf('ファイルを読み込み中: %s\n', csvFileName);
        data = readtable(dataPath);
        varNames = data.Properties.VariableNames;
        numVars = length(varNames);
        numPoints = height(data);
        
        % 時間軸（100Hz仮定）
        timeSec = (0:numPoints-1)' * 0.01;
        timeLabel = 'Time (s)';
        
        fprintf('  データ読込完了: %d行, %d列\n', numPoints, numVars);
        
        % 出力ファイル名のベース（拡張子除去）
        [~, csvBaseName, ~] = fileparts(csvFileName);
        
        % ファイル毎のサブディレクトリを作成
        fileResultsDir = fullfile(resultsDir, csvBaseName);
        if ~isfolder(fileResultsDir)
            mkdir(fileResultsDir);
            fprintf('  サブディレクトリを作成: %s\n', fileResultsDir);
        end
        
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
                outputPath = fullfile(fileResultsDir, sprintf('%s_%s.png', csvBaseName, safeVarName));
                saveas(fig, outputPath);
                close(fig);
                fprintf('    ✓ %s を保存\n', varName);
            end
        end
        fprintf('  ✓ %s の処理が完了しました（%d グラフ生成）\n\n', csvBaseName, numVars);
        
    catch ME
        fprintf('  エラー: %s\n\n', ME.message);
    end
end

fprintf('全CSVファイルのグラフ化が完了しました。\n');
