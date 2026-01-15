% CSVに全て0の行があるか調べるスクリプト
clear; clc;

csv_file = 'C:/Users/taki/Local/local/Matlab/OBS/RLS/motion_capture/csv/record_20260114_173422.csv';
data = readtable(csv_file);

% pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_wが全て0の行を検出
zero_mask = (data.pos_x == 0) & (data.pos_y == 0) & (data.pos_z == 0) & ...
            (data.quat_x == 0) & (data.quat_y == 0) & (data.quat_z == 0) & (data.quat_w == 0);
zero_idx = find(zero_mask);

if isempty(zero_idx)
    fprintf('全て0の行は見つかりませんでした。\n');
else
    fprintf('全て0の行が%d個見つかりました。\n', length(zero_idx));
    fprintf('最初の全て0の行番号: %d\n', zero_idx(1));
    disp(data(zero_idx(1), :));
end
