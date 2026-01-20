%% OBSV_data.csv の TimeUS フィールド詳細分析
% 100Hz仮定での妥当性を検証

dataPath = fullfile('..', 'data', 'OBSV_data.csv');

fprintf('\n');
fprintf('================================================================================\n');
fprintf('TimeUS フィールド詳細分析 - 100Hz仮定の検証\n');
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
    fprintf('乖離: 実データ行数は予想の %.4f倍\n\n', length(timeUS)/expectedNumPoints);
    
    % 連続データの時間差を分析
    fprintf('【TimeUS値の差分分析（最初の20行）】\n');
    fprintf('行番  TimeUS[i-1]       TimeUS[i]       差分(μs)      差分(ms)\n');
    fprintf('---------------------------------------------------------------------------\n');
    
    diffs = [];
    for i = 2:min(21, length(timeUS))
        diff = timeUS(i) - timeUS(i-1);
        diffs = [diffs; diff];
        fprintf('%3d  %.4e  %.4e  %.4e  %12.2f\n', i, timeUS(i-1), timeUS(i), diff, diff/1000);
    end
    
    % データが時系列順になっているか確認
    fprintf('\n【時系列順序の確認】\n');
    timeDiff = diff(timeUS);
    isMonotonic = all(timeDiff >= 0);
    fprintf('厳密に昇順: %s\n', logical2str(isMonotonic));
    
    % より正確に検出
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
    fprintf('\n最初の10セクションの統計:\n');
    fprintf('セクション  開始行  終了行   点数    時間範囲(μs)   平均周期(μs)   周期(ms)\n');
    fprintf('------------------------------------------------------------------------\n');
    
    for idx = 1:min(10, size(sections, 1))
        startRow = sections(idx, 1);
        endRow = sections(idx, 2);
        numPoints = endRow - startRow + 1;
        
        if numPoints > 1
            timeRange = timeUS(endRow) - timeUS(startRow);
            avgPeriod = timeRange / (numPoints - 1);
            periodMS = avgPeriod / 1000;
        else
            timeRange = 0;
            avgPeriod = 0;
            periodMS = 0;
        end
        
        fprintf('%4d       %5d  %5d  %5d    %.4e    %.4e    %8.3f\n', ...
            idx, startRow, endRow, numPoints, timeRange, avgPeriod, periodMS);
    end
    
    % 結論
    fprintf('\n');
    fprintf('================================================================================\n');
    fprintf('【結論と考察】\n');
    fprintf('================================================================================\n\n');
    
    fprintf('1. TimeUS値の異常性\n');
    fprintf('   ・TimeUS = 1.5e15～2.0e15という極めて大きな値\n');
    fprintf('   ・これは総データ期間が約5600日（15年以上）を意味する\n');
    fprintf('   ・推測: TimeUSは標準的なマイクロ秒タイムスタンプではなく、\n');
    fprintf('           別の形式データか、下位ビットが破損している可能性\n\n');
    
    fprintf('2. データの時系列順序\n');
    fprintf('   ・TimeUSが昇順になっていない（複数の昇順セクション存在）\n');
    fprintf('   ・異なるセッション/試行のデータが混在している可能性\n');
    fprintf('   ・または、データログのシャッフルが発生した可能性\n\n');
    
    fprintf('3. 100Hzサンプリングの妥当性\n');
    fprintf('   ・期待周期: 10,000 μs (10 ms)\n');
    fprintf('   ・実測: 各セクション内の周期をチェック必要\n');
    fprintf('   ・TimeUS値の乖離が大きすぎるため、100Hz仮定は\n');
    fprintf('     TimeUSから検証できない\n\n');
    
    fprintf('4. 推奨される対応\n');
    fprintf('   ・TimeUSではなく、行番号を時間軸として使用\n');
    fprintf('   ・100Hzサンプリングであれば、行i = (i-1) * 10 ms\n');
    fprintf('   ・セクション内での相対時刻の計算\n');
    fprintf('   ・データソースの仕様書を確認してTimeUS形式を特定\n\n');
    
    fprintf('================================================================================\n\n');
    
catch ME
    fprintf('エラー: %s\n', ME.message);
end

function str = logical2str(val)
    if val
        str = '○ Yes';
    else
        str = '✗ No';
    end
end
