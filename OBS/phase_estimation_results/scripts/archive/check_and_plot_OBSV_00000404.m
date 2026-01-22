%% OBSV_data_00000404.csv の確認とグラフ作成

clear; close all;

% データファイルパス
dataPath = fullfile('..', 'data', 'OBSV_data_00000404.csv');

fprintf('\n');
fprintf('================================================================================\n');
fprintf('OBSV_data_00000404.csv の確認とグラフ作成\n');
fprintf('================================================================================\n\n');

try
    % データ読み込み
    fprintf('ファイルを読み込み中...\n');
    data = readtable(dataPath);
    
    % テーブル情報の表示
    fprintf('\n【データ構造の確認】\n');
    fprintf('総行数: %d\n', height(data));
    fprintf('カラム数: %d\n\n', width(data));
    
    % カラム名の表示
    fprintf('カラム名一覧:\n');
    varNames = data.Properties.VariableNames;
    for i = 1:length(varNames)
        fprintf('  %2d: %s\n', i, varNames{i});
    end
    fprintf('\n');
    
    % TimeUS の確認
    if ismember('TimeUS', varNames)
        fprintf('【TimeUS フィールドの確認】\n');
        timeUS = data.TimeUS;
        fprintf('データ点数: %d\n', length(timeUS));
        fprintf('最小値: %.6e\n', min(timeUS));
        fprintf('最大値: %.6e\n', max(timeUS));
        fprintf('範囲: %.6e\n', max(timeUS) - min(timeUS));
        
        % TimeUS差分の確認
        timeDiff = diff(timeUS);
        posDiff = timeDiff(timeDiff > 0);
        fprintf('正の差分 - 最小: %.3f, 最大: %.3f, 平均: %.3f μs\n\n', ...
            min(posDiff), max(posDiff), mean(posDiff));
        
        % 時間軸を作成
        % timeUS が利用可能か確認 (平均差分が1000μs以下なら有効と判断)
        if ~isempty(posDiff) && mean(posDiff) < 1000 && mean(posDiff) > 0
            fprintf('✓ TimeUS は有効です。マイクロ秒タイムスタンプとして使用します\n\n');
            timeSec = (timeUS - timeUS(1)) / 1e6;
            timeLabel = 'Time (s) from TimeUS';
        else
            % TimeUS が使えない場合は行番号を使用
            fprintf('⚠️  TimeUS 値が不正です。行番号を時間軸として使用します\n');
            timeSec = (0:length(timeUS)-1)' * 0.01;  % 100Hz仮定
            timeLabel = 'Time (s) - 100Hz assumed (index-based)';
        end
        
        fprintf('【時間軸を作成】\n');
        fprintf('時間範囲: %.2f 秒\n\n', max(timeSec) - min(timeSec));
    else
        fprintf('⚠️  TimeUS フィールドが見つかりません\n');
        fprintf('行番号を時間軸として使用します (100Hz仮定)\n\n');
        timeSec = (0:height(data)-1)' * 0.01;
        timeLabel = 'Time (s) - 100Hz assumed (index-based)';
    end
    
    % 利用可能な数値カラムを検出
    fprintf('【利用可能な数値カラン】\n');
    numericVars = [];
    for i = 1:length(varNames)
        if isnumeric(data.(varNames{i}))
            numericVars = [numericVars, i];
            fprintf('  %s\n', varNames{i});
        end
    end
    fprintf('\n');
    
    % グラフ作成
    fprintf('【グラフを作成中...】\n');
    
    % TimeUS 以外の数値カラムをプロット
    plotVars = setdiff(numericVars, find(ismember(varNames, 'TimeUS')));
    
    if ~isempty(plotVars)
        % サブプロット数の決定
        numPlots = length(plotVars);
        if numPlots > 9
            numPlots = 9;  % 最大9プロット
            fprintf('⚠️  9個以上のカラムがあるため、最初の9個をプロット\n');
        end
        
        % グリッドレイアウトの決定
        numRows = ceil(sqrt(numPlots));
        numCols = ceil(numPlots / numRows);
        
        % グラフウィンドウを作成
        fig = figure('Position', [100, 100, 1200, 800], 'Name', 'OBSV_data_00000404 Analysis');
        
        for idx = 1:numPlots
            varIdx = plotVars(idx);
            varName = varNames{varIdx};
            
            subplot(numRows, numCols, idx);
            plot(timeSec, data.(varName), 'LineWidth', 1.5);
            
            grid on;
            xlabel(timeLabel);
            ylabel(varName);
            title(sprintf('%s (%d points)', varName, height(data)));
        end
        
        % 全体のタイトル
        sgtitle('OBSV_data_00000404.csv Analysis', 'FontSize', 14, 'FontWeight', 'bold');
        
        % レイアウト調整
        drawnow;
        
        % グラフを保存
        outputPath = fullfile('..', 'results', 'OBSV_00000404_plots.png');
        if ~isfolder(fullfile('..', 'results'))
            mkdir(fullfile('..', 'results'));
        end
        saveas(fig, outputPath);
        fprintf('✓ グラフを保存: %s\n', outputPath);
        
    else
        fprintf('⚠️  プロット可能な数値カラムがありません\n');
    end
    
    fprintf('\n');
    fprintf('================================================================================\n');
    fprintf('処理完了\n');
    fprintf('================================================================================\n\n');
    
catch ME
    fprintf('エラー: %s\n', ME.message);
    fprintf('Stack trace:\n');
    fprintf('%s\n', ME.getReport());
end
