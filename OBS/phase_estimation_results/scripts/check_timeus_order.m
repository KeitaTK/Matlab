%% OBSV_data_00000404.csv のTimeUS（タイムスタンプ）昇順チェック
clear; clc;

% データファイルパス
dataPath = fullfile('..', 'data', 'OBSV_data_00000404.csv');

try
    fprintf('ファイルを読み込み中...\n');
    data = readtable(dataPath);
    timeUS = data.TimeUS;
    
    % 差分を計算
    timeDiff = diff(timeUS);
    
    % 昇順でない箇所を検出
    notAscendingIdx = find(timeDiff < 0);
    
    if isempty(notAscendingIdx)
        fprintf('✓ TimeUSは全て昇順に並んでいます。\n');
    else
        fprintf('⚠️ TimeUSが昇順でない箇所が%d箇所あります。\n', length(notAscendingIdx));
        fprintf('例: 最初の5箇所\n');
        disp([notAscendingIdx(1:min(5,end)), timeUS(notAscendingIdx(1:min(5,end))), timeUS(notAscendingIdx(1:min(5,end))+1)]);
    end
    
    fprintf('総データ数: %d\n', length(timeUS));
    fprintf('昇順でない箇所数: %d\n', length(notAscendingIdx));
catch ME
    fprintf('エラー: %s\n', ME.message);
end
