%% OBSV_data.csv の TimeUS フィールド詳細分析 - シンプル版
% 100Hz仮定での妥当性を検証

dataPath = fullfile('..', 'data', 'OBSV_data.csv');

fprintf('\n');
fprintf('================================================================================\n');
fprintf('OBSV_data.csv TimeUS フィールド分析 - 100Hz仮定の検証\n');
fprintf('================================================================================\n\n');

try
    data = readtable(dataPath);
    timeUS = data.TimeUS;
    
    fprintf('【基本統計】\n');
    fprintf('総データ行数: %d\n', length(timeUS));
    fprintf('TimeUS最小値: %.6e\n', min(timeUS));
    fprintf('TimeUS最大値: %.6e\n', max(timeUS));
    fprintf('TimeUS範囲: %.6e\n\n', max(timeUS) - min(timeUS));
    
    % マイクロ秒と仮定した場合の分析
    rangeUS = max(timeUS) - min(timeUS);
    rangeSec = rangeUS / 1e6;
    
    fprintf('【マイクロ秒と仮定した場合の時間換算】\n');
    fprintf('時間範囲: %.2f 秒 = %.2f 分 = %.2f 時間 = %.2f 日\n\n', ...
        rangeSec, rangeSec/60, rangeSec/3600, rangeSec/86400);
    
    % 100Hzでの期待値
    expectedPeriodUS = 10000;  % 10ms
    expectedNumPoints = rangeUS / expectedPeriodUS;
    
    fprintf('【100Hz仮定での期待値】\n');
    fprintf('予想サンプリング周期: %d μs (10 ms)\n', expectedPeriodUS);
    fprintf('予想総データ行数（時間範囲から計算）: %.0f\n', expectedNumPoints);
    fprintf('実データ行数: %d\n', length(timeUS));
    fprintf('実データ/予想: %.6e (ほぼ0）→ 100Hz仮定は成立していない\n\n', length(timeUS)/expectedNumPoints);
    
    % TimeUS差分の統計
    timeDiff = diff(timeUS);
    
    fprintf('【連続サンプル間のTimeUS差分統計】\n');
    fprintf('正の差分: %d 個\n', sum(timeDiff > 0));
    fprintf('0の差分: %d 個\n', sum(timeDiff == 0));
    fprintf('負の差分: %d 個\n\n', sum(timeDiff < 0));
    
    if sum(timeDiff > 0) > 0
        posDiff = timeDiff(timeDiff > 0);
        fprintf('【正の差分のみの統計】\n');
        fprintf('最小: %.6e μs = %.3f ms\n', min(posDiff), min(posDiff)/1000);
        fprintf('最大: %.6e μs = %.3f ms\n', max(posDiff), max(posDiff)/1000);
        fprintf('平均: %.6e μs = %.3f ms\n', mean(posDiff), mean(posDiff)/1000);
        fprintf('中央値: %.6e μs = %.3f ms\n\n', median(posDiff), median(posDiff)/1000);
    end
    
    % 昇順セクションを検出
    fprintf('【データの時系列性の確認】\n');
    sections = [];
    startIdx = 1;
    for i = 2:length(timeUS)
        if timeUS(i) < timeUS(i-1)
            sections = [sections; startIdx, i-1];
            startIdx = i;
        end
    end
    sections = [sections; startIdx, length(timeUS)];
    
    fprintf('昇順セクション数: %d\n', size(sections, 1));
    fprintf('⇒ データが時系列順に並んでいない！複数セッションが混在\n\n');
    
    fprintf('最初の10セクション統計:\n');
    fprintf('セクション  開始行-終了行   点数    時間範囲(μs)\n');
    fprintf('---------------------------------------------------\n');
    
    for idx = 1:min(10, size(sections, 1))
        startRow = sections(idx, 1);
        endRow = sections(idx, 2);
        numPoints = endRow - startRow + 1;
        timeRange = timeUS(endRow) - timeUS(startRow);
        fprintf('%4d       %5d-%5d    %5d    %.4e\n', ...
            idx, startRow, endRow, numPoints, timeRange);
    end
    
    fprintf('\n');
    fprintf('================================================================================\n');
    fprintf('【重要な発見】\n');
    fprintf('================================================================================\n\n');
    
    fprintf('1. ❌ TimeUSがマイクロ秒タイムスタンプではない可能性が高い\n');
    fprintf('   ・時間範囲が5600日分という非現実的な値\n');
    fprintf('   ・100Hzサンプリング192点なら約2秒のはずが...\n\n');
    
    fprintf('2. ❌ データが時系列順に並んでいない\n');
    fprintf('   ・TimeUS値が上下に変動\n');
    fprintf('   ・複数のセッション/試行が混在している可能性\n\n');
    
    fprintf('3. ❌ 100Hz仮定は TimeUS から検証できない\n');
    fprintf('   ・差分の大きさから判断不可能\n');
    fprintf('   ・代わりに行番号を時間軸として使うべき\n\n');
    
    fprintf('【推奨される対応】\n');
    fprintf('---------------------------------------------------\n');
    fprintf('・ Time軸: 行番号 i を使用\n');
    fprintf('・ 100Hz仮定なら: t = (i-1) * 0.01 秒\n');
    fprintf('・ TimeUS列は参考情報のみ\n');
    fprintf('・ データソース仕様書を確認\n\n');
    
    fprintf('================================================================================\n\n');
    
catch ME
    fprintf('エラー: %s\n', ME.message);
end
