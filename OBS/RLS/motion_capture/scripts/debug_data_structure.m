% データ構造の詳細確認スクリプト
clear all; close all; clc;

csv_file = 'C:/Users/taki/Local/local/Matlab/OBS/RLS/motion_capture/csv/record_20260114_173422.csv';
data = readtable(csv_file);

% 最初の100行を確認
fprintf('=== データの最初の100行 ===\n');
disp(head(data, 100));

fprintf('\n=== タイムスタンプの統計 ===\n');
fprintf('全タイムスタンプの一意な値の個数: %d\n', length(unique(data.timestamp)));
fprintf('タイムスタンプの最小値: %f\n', min(data.timestamp));
fprintf('タイムスタンプの最大値: %f\n', max(data.timestamp));

fprintf('\n=== rigid_body_idごとのデータ数 ===\n');
drone_data = data(data.rigid_body_id == 1, :);
payload_data = data(data.rigid_body_id == 2, :);
fprintf('rigid_body_id=1 (ドローン): %d行\n', height(drone_data));
fprintf('rigid_body_id=2 (ペイロード): %d行\n', height(payload_data));

fprintf('\n=== rigid_body_id=1の最初の20行 ===\n');
disp(head(drone_data, 20));

fprintf('\n=== rigid_body_id=2の最初の20行 ===\n');
disp(head(payload_data, 20));

fprintf('\n=== タイムスタンプが異なる行をカウント ===\n');
% ドローンとペイロードのタイムスタンプを時系列順に取得
drone_timestamps = drone_data.timestamp;
payload_timestamps = payload_data.timestamp;
fprintf('ドローンのタイムスタンプの一意な値: %d個\n', length(unique(drone_timestamps)));
fprintf('ペイロードのタイムスタンプの一意な値: %d個\n', length(unique(payload_timestamps)));

fprintf('\n=== 行の並び順の確認（最初の50行） ===\n');
fprintf('行番号\tID\tpos_x\ttimestamp\n');
for i = 1:50
    fprintf('%d\t%d\t%.4f\t%f\n', i, data.rigid_body_id(i), data.pos_x(i), data.timestamp(i));
end

fprintf('\n=== 最初の10行のドローンとペイロード座標 ===\n');
fprintf('行\tID\tpos_x\tpos_y\tpos_z\n');
for i = 1:10
    fprintf('%d\t%d\t%.4f\t%.4f\t%.4f\n', i, data.rigid_body_id(i), data.pos_x(i), data.pos_y(i), data.pos_z(i));
end
