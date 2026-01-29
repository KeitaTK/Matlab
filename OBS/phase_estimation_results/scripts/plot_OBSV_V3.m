%% OBSV_data の全カラム可視化（複数ファイル対応・SW列対応版）

clear; close all;

% === 設定 ===
dataDir = fullfile('..', 'data');
resultsDir = fullfile('..', 'results');

% 処理対象のCSVファイルを指定 （このセルアレイを編集してください）
csvFileNames = {
    'OBSV_data_00000443.csv'
    'OBSV_data_00000444.csv'
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
        
        % グループ化されたグラフを作成
        fprintf('  グループグラフを作成中...\n');
        
        % 位置データ (PLX, PLY, PLZ)
        posVars = {'PLX', 'PLY', 'PLZ'};
        availablePos = intersect(posVars, varNames);
        if ~isempty(availablePos)
            figPos = figure('Visible','off','Position',[100,100,1200,800]);
            for i = 1:length(availablePos)
                subplot(length(availablePos), 1, i);
                plot(timeSec, data.(availablePos{i}), 'LineWidth', 1.5);
                ylabel(availablePos{i});
                xlabel(timeLabel);
                title(sprintf('%s vs Time', availablePos{i}));
                grid on;
            end
            saveas(figPos, fullfile(fileResultsDir, sprintf('%s_position_data.png', csvBaseName)));
            close(figPos);
            fprintf('    ✓ 位置データグラフを保存\n');
        end

        % PLXとAX,AYから計算した振幅を重ねてプロット
        if ismember('PLX', varNames) && ismember('AX', varNames) && ismember('AY', varNames)
            amplitude = sqrt(data.AX.^2 + data.AY.^2);
            figPLXamp = figure('Visible','off','Position',[100,100,1200,600]);
            subplot(2,1,1);
            plot(timeSec, data.PLX, 'b-', 'LineWidth', 1.5);
            ylabel('PLX');
            xlabel(timeLabel);
            title('PLX vs Time');
            grid on;
            subplot(2,1,2);
            plot(timeSec, amplitude, 'r--', 'LineWidth', 1.5);
            ylabel('sqrt(AX^2 + AY^2)');
            xlabel(timeLabel);
            title('AX,AYから計算した振幅 vs Time');
            grid on;
            saveas(figPLXamp, fullfile(fileResultsDir, sprintf('%s_PLX_and_Amplitude.png', csvBaseName)));
            close(figPLXamp);
            fprintf('    ✓ PLXと振幅グラフを保存\n');
        end
        
        % A系列 (AX, AY)
        aVars = {'AX', 'AY'};
        availableA = intersect(aVars, varNames);
        if ~isempty(availableA) && any(data.(availableA{1}) ~= 0 | data.(availableA{end}) ~= 0)
            figA = figure('Visible','off','Position',[100,100,1200,600]);
            for i = 1:length(availableA)
                subplot(length(availableA), 1, i);
                plot(timeSec, data.(availableA{i}), 'LineWidth', 1.5);
                ylabel(availableA{i});
                xlabel(timeLabel);
                title(sprintf('%s vs Time', availableA{i}));
                grid on;
            end
            saveas(figA, fullfile(fileResultsDir, sprintf('%s_A_series.png', csvBaseName)));
            close(figA);
            fprintf('    ✓ A系列グラフを保存\n');
        end
        
        % B系列 (BX, BY)
        bVars = {'BX', 'BY'};
        availableB = intersect(bVars, varNames);
        if ~isempty(availableB) && any(data.(availableB{1}) ~= 0 | data.(availableB{end}) ~= 0)
            figB = figure('Visible','off','Position',[100,100,1200,600]);
            for i = 1:length(availableB)
                subplot(length(availableB), 1, i);
                plot(timeSec, data.(availableB{i}), 'LineWidth', 1.5);
                ylabel(availableB{i});
                xlabel(timeLabel);
                title(sprintf('%s vs Time', availableB{i}));
                grid on;
            end
            saveas(figB, fullfile(fileResultsDir, sprintf('%s_B_series.png', csvBaseName)));
            close(figB);
            fprintf('    ✓ B系列グラフを保存\n');
        end
        
        % C系列 (CX, CY)
        cVars = {'CX', 'CY'};
        availableC = intersect(cVars, varNames);
        if ~isempty(availableC) && any(data.(availableC{1}) ~= 0 | data.(availableC{end}) ~= 0)
            figC = figure('Visible','off','Position',[100,100,1200,600]);
            for i = 1:length(availableC)
                subplot(length(availableC), 1, i);
                plot(timeSec, data.(availableC{i}), 'LineWidth', 1.5);
                ylabel(availableC{i});
                xlabel(timeLabel);
                title(sprintf('%s vs Time', availableC{i}));
                grid on;
            end
            saveas(figC, fullfile(fileResultsDir, sprintf('%s_C_series.png', csvBaseName)));
            close(figC);
            fprintf('    ✓ C系列グラフを保存\n');
        end
        

        % 制御パラメータ (F, P, X, Y)
        ctrlVars = {'F', 'P', 'X', 'Y'};
        availableCtrl = intersect(ctrlVars, varNames);
        % 紐の長さ1.04mの単振り子の理論振動周波数を計算
        g = 9.80665; % 重力加速度 [m/s^2]
        L = 1.04;    % 紐の長さ [m]
        f_pendulum = 1/(2*pi)*sqrt(g/L); % [Hz]
        if ~isempty(availableCtrl)
            figCtrl = figure('Visible','off','Position',[100,100,1200,1000]);
            for i = 1:length(availableCtrl)
                subplot(length(availableCtrl), 1, i);
                plot(timeSec, data.(availableCtrl{i}), 'LineWidth', 1.5);
                ylabel(availableCtrl{i});
                xlabel(timeLabel);
                title(sprintf('%s vs Time', availableCtrl{i}));
                grid on;
                % Fのときだけ赤横線を描画
                if strcmp(availableCtrl{i}, 'F')
                    yline(f_pendulum, 'r-', 'LineWidth', 2, 'Label', sprintf('%.3f Hz (L=1.04m)', f_pendulum), 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'bottom');
                end
            end
            saveas(figCtrl, fullfile(fileResultsDir, sprintf('%s_control_params.png', csvBaseName)));
            close(figCtrl);
            fprintf('    ✓ 制御パラメータグラフを保存\n');
        end
        
        % SW (スイッチ状態)
        if ismember('SW', varNames)
            figSW = figure('Visible','off','Position',[100,100,1200,400]);
            plot(timeSec, data.SW, 'LineWidth', 2);
            ylabel('SW');
            xlabel(timeLabel);
            title('Switch State vs Time');
            grid on;
            ylim([-0.5, max(data.SW) + 0.5]);
            saveas(figSW, fullfile(fileResultsDir, sprintf('%s_SW.png', csvBaseName)));
            close(figSW);
            fprintf('    ✓ SWグラフを保存\n');
        end
        
        % 全データ概観
        figAll = figure('Visible','off','Position',[100,100,1400,1000]);
        plotVars = setdiff(varNames, {'TimeUS'});
        nVars = length(plotVars);
        nCols = 4;
        nRows = ceil(nVars / nCols);
        
        for i = 1:nVars
            subplot(nRows, nCols, i);
            plot(timeSec, data.(plotVars{i}), 'LineWidth', 1);
            ylabel(plotVars{i});
            xlabel(timeLabel);
            title(plotVars{i});
            grid on;
        end
        saveas(figAll, fullfile(fileResultsDir, sprintf('%s_all_data.png', csvBaseName)));
        close(figAll);
        fprintf('    ✓ 全データ概観を保存\n');
        
        fprintf('  ✓ %s の処理が完了しました\n\n', csvBaseName);
        
    catch ME
        fprintf('  エラー: %s\n\n', ME.message);
    end
end

fprintf('全CSVファイルのグラフ化が完了しました。\n');
fprintf('保存先: %s\n', resultsDir);
