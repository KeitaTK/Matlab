%% OBSV_data_00000404.csv の詳細分析とグラフ作成

clear; close all;

% データファイルパス
dataPath = fullfile('..', 'data', 'OBSV_data_00000404.csv');

fprintf('\n');
fprintf('================================================================================\n');
fprintf('OBSV_data_00000404.csv の詳細分析\n');
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
    
    % 時間軸を作成（行番号ベース、100Hz仮定）
    numPoints = height(data);
    timeSec = (0:numPoints-1)' * 0.01;  % 100Hz: 10ms per sample
    timeLabel = 'Time (s)';
    
    fprintf('【時間軸情報】\n');
    fprintf('総サンプル数: %d\n', numPoints);
    fprintf('サンプリング周期: 10 ms (100Hz仮定)\n');
    fprintf('総時間: %.2f 秒 = %.2f 分\n\n', max(timeSec), max(timeSec)/60);
    
    % TimeUS の情報
    fprintf('【TimeUS フィールド情報】\n');
    timeUS = data.TimeUS;
    fprintf('最小値: %.6e\n', min(timeUS));
    fprintf('最大値: %.6e\n', max(timeUS));
    fprintf('⚠️  TimeUS 値が不正なため、行番号ベースの時間軸を使用します\n\n');
    
    % グラフ1: 位置データ (PLX, PLY, PLZ)
    fprintf('【グラフ1を作成: 位置データ (PLX, PLY, PLZ)】\n');
    fig1 = figure('Position', [100, 100, 1200, 600], 'Name', 'Position Data');
    
    subplot(3,1,1);
    plot(timeSec, data.PLX, 'b-', 'LineWidth', 1);
    grid on; ylabel('PLX'); title('Position X');
    
    subplot(3,1,2);
    plot(timeSec, data.PLY, 'g-', 'LineWidth', 1);
    grid on; ylabel('PLY'); title('Position Y');
    
    subplot(3,1,3);
    plot(timeSec, data.PLZ, 'r-', 'LineWidth', 1);
    grid on; xlabel(timeLabel); ylabel('PLZ'); title('Position Z');
    
    sgtitle('OBSV Position Data (PLX, PLY, PLZ)', 'FontSize', 14, 'FontWeight', 'bold');
    
    % グラフを保存
    outputPath1 = fullfile('..', 'results', 'OBSV_00000404_position.png');
    saveas(fig1, outputPath1);
    fprintf('✓ グラフ1を保存: %s\n', outputPath1);
    
    % グラフ2: 加速度データ (AX, AY)
    fprintf('\n【グラフ2を作成: 加速度データ (AX, AY)】\n');
    fig2 = figure('Position', [100, 100, 1200, 600], 'Name', 'Acceleration Data');
    
    subplot(2,1,1);
    plot(timeSec, data.AX, 'b-', 'LineWidth', 1);
    grid on; ylabel('AX'); title('Acceleration X');
    
    subplot(2,1,2);
    plot(timeSec, data.AY, 'g-', 'LineWidth', 1);
    grid on; xlabel(timeLabel); ylabel('AY'); title('Acceleration Y');
    
    sgtitle('OBSV Acceleration Data (AX, AY)', 'FontSize', 14, 'FontWeight', 'bold');
    
    outputPath2 = fullfile('..', 'results', 'OBSV_00000404_acceleration.png');
    saveas(fig2, outputPath2);
    fprintf('✓ グラフ2を保存: %s\n', outputPath2);
    
    % グラフ3: 観測値 (BX, BY, CX, CY)
    fprintf('\n【グラフ3を作成: 観測値 (BX, BY, CX, CY)】\n');
    fig3 = figure('Position', [100, 100, 1200, 600], 'Name', 'Observation Data');
    
    subplot(2,2,1);
    plot(timeSec, data.BX, 'b-', 'LineWidth', 0.5);
    grid on; ylabel('BX'); title('Observation B - X');
    
    subplot(2,2,2);
    plot(timeSec, data.BY, 'g-', 'LineWidth', 0.5);
    grid on; ylabel('BY'); title('Observation B - Y');
    
    subplot(2,2,3);
    plot(timeSec, data.CX, 'r-', 'LineWidth', 0.5);
    grid on; xlabel(timeLabel); ylabel('CX'); title('Observation C - X');
    
    subplot(2,2,4);
    plot(timeSec, data.CY, 'm-', 'LineWidth', 0.5);
    grid on; xlabel(timeLabel); ylabel('CY'); title('Observation C - Y');
    
    sgtitle('OBSV Observation Data (BX, BY, CX, CY)', 'FontSize', 14, 'FontWeight', 'bold');
    
    outputPath3 = fullfile('..', 'results', 'OBSV_00000404_observations.png');
    saveas(fig3, outputPath3);
    fprintf('✓ グラフ3を保存: %s\n', outputPath3);
    
    % グラフ4: 推定値 (PRX, PRY, PRZ)
    fprintf('\n【グラフ4を作成: 推定値 (PRX, PRY, PRZ)】\n');
    fig4 = figure('Position', [100, 100, 1200, 600], 'Name', 'Estimated Position');
    
    subplot(3,1,1);
    plot(timeSec, data.PRX, 'b-', 'LineWidth', 1);
    grid on; ylabel('PRX'); title('Estimated Position X');
    
    subplot(3,1,2);
    plot(timeSec, data.PRY, 'g-', 'LineWidth', 1);
    grid on; ylabel('PRY'); title('Estimated Position Y');
    
    subplot(3,1,3);
    plot(timeSec, data.PRZ, 'r-', 'LineWidth', 1);
    grid on; xlabel(timeLabel); ylabel('PRZ'); title('Estimated Position Z');
    
    sgtitle('OBSV Estimated Position (PRX, PRY, PRZ)', 'FontSize', 14, 'FontWeight', 'bold');
    
    outputPath4 = fullfile('..', 'results', 'OBSV_00000404_estimated_position.png');
    saveas(fig4, outputPath4);
    fprintf('✓ グラフ4を保存: %s\n', outputPath4);
    
    % グラフ5: エラーと相関 (ERR, EST_FREQ, CORR)
    fprintf('\n【グラフ5を作成: エラーと統計情報 (ERR, EST_FREQ, CORR)】\n');
    fig5 = figure('Position', [100, 100, 1200, 600], 'Name', 'Error and Statistics');
    
    subplot(3,1,1);
    plot(timeSec, data.ERR, 'r-', 'LineWidth', 0.5);
    grid on; ylabel('Error'); title('Estimation Error');
    
    subplot(3,1,2);
    plot(timeSec, data.EST_FREQ, 'b-', 'LineWidth', 0.5);
    grid on; ylabel('Frequency (Hz)'); title('Estimated Frequency');
    
    subplot(3,1,3);
    plot(timeSec, data.CORR, 'g-', 'LineWidth', 0.5);
    grid on; xlabel(timeLabel); ylabel('Correlation'); title('Correlation Coefficient');
    
    sgtitle('OBSV Error and Statistics (ERR, EST_FREQ, CORR)', 'FontSize', 14, 'FontWeight', 'bold');
    
    outputPath5 = fullfile('..', 'results', 'OBSV_00000404_error_stats.png');
    saveas(fig5, outputPath5);
    fprintf('✓ グラフ5を保存: %s\n', outputPath5);
    
    % 統計情報の表示
    fprintf('\n【データ統計情報】\n');
    fprintf('PLX: 平均=%.4f, 最小=%.4f, 最大=%.4f\n', mean(data.PLX), min(data.PLX), max(data.PLX));
    fprintf('PLY: 平均=%.4f, 最小=%.4f, 最大=%.4f\n', mean(data.PLY), min(data.PLY), max(data.PLY));
    fprintf('PLZ: 平均=%.4f, 最小=%.4f, 最大=%.4f\n', mean(data.PLZ), min(data.PLZ), max(data.PLZ));
    fprintf('AX:  平均=%.4f, 最小=%.4f, 最大=%.4f\n', mean(data.AX), min(data.AX), max(data.AX));
    fprintf('AY:  平均=%.4f, 最小=%.4f, 最大=%.4f\n', mean(data.AY), min(data.AY), max(data.AY));
    fprintf('ERR: 平均=%.4f, 最小=%.4f, 最大=%.4f\n', mean(data.ERR), min(data.ERR), max(data.ERR));
    fprintf('EST_FREQ: 平均=%.4f, 最小=%.4f, 最大=%.4f\n', mean(data.EST_FREQ), min(data.EST_FREQ), max(data.EST_FREQ));
    fprintf('CORR: 平均=%.4f, 最小=%.4f, 最大=%.4f\n', mean(data.CORR), min(data.CORR), max(data.CORR));
    
    fprintf('\n');
    fprintf('================================================================================\n');
    fprintf('処理完了！\n');
    fprintf('生成されたグラフ：\n');
    fprintf('  1. %s\n', outputPath1);
    fprintf('  2. %s\n', outputPath2);
    fprintf('  3. %s\n', outputPath3);
    fprintf('  4. %s\n', outputPath4);
    fprintf('  5. %s\n', outputPath5);
    fprintf('================================================================================\n\n');
    
    % ウィンドウを全て表示
    drawnow;
    
catch ME
    fprintf('エラー: %s\n', ME.message);
    fprintf('Stack trace:\n');
    fprintf('%s\n', ME.getReport());
end
