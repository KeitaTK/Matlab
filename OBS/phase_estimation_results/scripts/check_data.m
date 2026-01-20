%% OBSV_data.csv のデータ内容を確認するスクリプト
% データファイルのパス
dataPath = fullfile('..', 'data', 'OBSV_data.csv');

fprintf('================================================================================\n');
fprintf('OBSV_data.csv データ確認\n');
fprintf('================================================================================\n\n');

try
    % CSVファイルを読み込む
    data = readtable(dataPath);
    
    fprintf('✓ ファイル読み込み成功\n');
    fprintf('  データ行数: %d\n', height(data));
    fprintf('  カラム数: %d\n', width(data));
    
    fprintf('\n--------------------------------------------------------------------------------\n');
    fprintf('カラム名:\n');
    fprintf('--------------------------------------------------------------------------------\n');
    varNames = data.Properties.VariableNames;
    for i = 1:length(varNames)
        fprintf('%2d. %s\n', i, varNames{i});
    end
    
    fprintf('\n--------------------------------------------------------------------------------\n');
    fprintf('各カラムの統計情報:\n');
    fprintf('--------------------------------------------------------------------------------\n');
    
    for i = 1:length(varNames)
        colName = varNames{i};
        colData = data.(colName);
        
        % NaN以外の値を取得
        validData = colData(~isnan(colData));
        
        if ~isempty(validData)
            fprintf('\n%s:\n', colName);
            fprintf('  非NaN値数: %d\n', length(validData));
            fprintf('  最小値: %.6e\n', min(validData));
            fprintf('  最大値: %.6e\n', max(validData));
            fprintf('  平均値: %.6e\n', mean(validData));
            fprintf('  標準偏差: %.6e\n', std(validData));
        end
    end
    
    fprintf('\n--------------------------------------------------------------------------------\n');
    fprintf('最初の5行:\n');
    fprintf('--------------------------------------------------------------------------------\n');
    disp(data(1:min(5, height(data)), :));
    
    fprintf('\n--------------------------------------------------------------------------------\n');
    fprintf('欠損値の確認:\n');
    fprintf('--------------------------------------------------------------------------------\n');
    for i = 1:length(varNames)
        colName = varNames{i};
        nanCount = sum(isnan(data.(colName)));
        fprintf('%s: %d 個のNaN\n', colName, nanCount);
    end
    
    % TimeUSを時間に変換してみる
    if ismember('TimeUS', varNames)
        fprintf('\n--------------------------------------------------------------------------------\n');
        fprintf('時間情報:\n');
        fprintf('--------------------------------------------------------------------------------\n');
        timeUS = data.TimeUS(~isnan(data.TimeUS));
        if length(timeUS) > 1
            timeRange = (max(timeUS) - min(timeUS)) / 1e6; % マイクロ秒から秒へ
            fprintf('  時間範囲: %.2f 秒\n', timeRange);
            fprintf('  開始時刻: %d\n', min(timeUS));
            fprintf('  終了時刻: %d\n', max(timeUS));
        end
    end
    
    fprintf('\n================================================================================\n');
    fprintf('データ確認完了\n');
    fprintf('================================================================================\n');
    
catch ME
    fprintf('\n✗ エラー発生: %s\n', ME.message);
    fprintf('スタックトレース:\n');
    for i = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
end
