% ドローン（rigid_body_id=1）のタイムスタンプ間隔・サンプリング周波数を確認
clear all; close all; clc;

csv_file = 'OBS/RLS/motion_capture/record_20260113_183814.csv';
data = readtable(csv_file);

drone_data = data(data.rigid_body_id == 1, :);
drone_time = drone_data.timestamp;

dt = diff(drone_time); % サンプリング間隔
mean_dt = mean(dt);
std_dt = std(dt);

fs = 1/mean_dt;

fprintf('===== ドローンのサンプリング間隔・周波数 =====\n');
fprintf('平均サンプリング間隔: %.6f s\n', mean_dt);
fprintf('サンプリング間隔の標準偏差: %.6f s\n', std_dt);
fprintf('推定サンプリング周波数: %.2f Hz\n', fs);

fprintf('サンプリング間隔の最小: %.6f, 最大: %.6f\n', min(dt), max(dt));

% ヒストグラムで分布も確認
figure;
histogram(dt, 50);
xlabel('サンプリング間隔 [s]');
ylabel('出現数');
title('ドローンのサンプリング間隔分布');
