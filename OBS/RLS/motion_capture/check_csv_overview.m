% CSVの中身を確認するスクリプト
clear all; close all; clc;

csv_file = 'OBS/RLS/motion_capture/record_20260113_183814.csv';
data = readtable(csv_file);

fprintf('===== CSV ファイルの先頭5行 =====\n');
disp(data(1:min(5, height(data)), :));

fprintf('===== rigid_body_idごとの統計 =====\n');
ids = unique(data.rigid_body_id);
for i = 1:length(ids)
    id = ids(i);
    idx = data.rigid_body_id == id;
    t = data.timestamp(idx);
    fprintf('ID=%d: サンプル数=%d, timestamp範囲=[%.3f, %.3f]\n', id, sum(idx), min(t), max(t));
end

fprintf('===== timestamp全体範囲 =====\n');
fprintf('[%.3f, %.3f]\n', min(data.timestamp), max(data.timestamp));

% 必要なら全体のカラム名も表示
fprintf('===== カラム名一覧 =====\n');
disp(data.Properties.VariableNames);