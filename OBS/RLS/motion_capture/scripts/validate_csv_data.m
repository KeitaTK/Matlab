% CSVデータの検証プログラム
% ドローンとペイロードのタイムスタンプの重なりを確認

clear all; close all; clc;

% CSVファイルのパス
csv_file = 'OBS\RLS\motion_capture\record_20260113_183814.csv';
data = readtable(csv_file);

% ドローンとペイロードのデータを分離
drone_data = data(data.rigid_body_id == 1, :);
payload_data = data(data.rigid_body_id == 2, :);

% タイムスタンプの範囲を取得
drone_time_range = [min(drone_data.timestamp), max(drone_data.timestamp)];
payload_time_range = [min(payload_data.timestamp), max(payload_data.timestamp)];

% 結果を表示
fprintf('Drone time range: [%.3f, %.3f] s\n', drone_time_range(1), drone_time_range(2));
fprintf('Payload time range: [%.3f, %.3f] s\n', payload_time_range(1), payload_time_range(2));

% 重なりの確認
if drone_time_range(2) < payload_time_range(1) || drone_time_range(1) > payload_time_range(2)
    fprintf('Error: No overlapping time range between drone and payload data.\n');
else
    fprintf('Success: Time ranges overlap.\n');
end

% Exit MATLAB after execution
exit;