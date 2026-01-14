% Motion Capture Data Plotting
% Plots x-axis position data for drone (rigid_body_id=1) and payload (rigid_body_id=2)
% Uses linear interpolation for payload data to match drone timestamps


clear all; close all; clc;

% ファイルとゲインの対応リスト
file_gain_list = {
    'OBS/RLS/motion_capture/record_20260113_183814.csv', 0;
    'OBS/RLS/motion_capture/record_20260113_183850.csv', 0;
    'OBS/RLS/motion_capture/record_20260113_185207.csv', 0.02;
    'OBS/RLS/motion_capture/record_20260113_185232.csv', 0.02;
    'OBS/RLS/motion_capture/record_20260113_185602.csv', 0.04;
    'OBS/RLS/motion_capture/record_20260113_185628.csv', 0.04;
    'OBS/RLS/motion_capture/record_20260113_185924.csv', 0.06;
    'OBS/RLS/motion_capture/record_20260113_190008.csv', 0.06;
    'OBS/RLS/motion_capture/record_20260113_190233.csv', 0.08;
    'OBS/RLS/motion_capture/record_20260113_190300.csv', 0.08;
    'OBS/RLS/motion_capture/record_20260113_190600.csv', 0;
    'OBS/RLS/motion_capture/record_20260113_190626.csv', 0;
};



% ゲイン値で昇順ソート
[~, idx_sort] = sort(cell2mat(file_gain_list(:,2)));
file_gain_list_sorted = file_gain_list(idx_sort, :);


color_list = lines(size(file_gain_list_sorted,1));
figure('Position', [100, 100, 1600, 600]);

all_diff_x_min = inf;
all_diff_x_max = -inf;
all_diff_x = cell(size(file_gain_list_sorted,1),1);

% まず全データのdiff_x範囲を取得

all_time_max = 0;
for i = 1:size(file_gain_list_sorted,1)
    csv_file = file_gain_list_sorted{i,1};
    data = readtable(csv_file);
    drone_data = data(data.rigid_body_id == 1, :);
    payload_data = data(data.rigid_body_id == 2, :);
    N = height(drone_data);
    fs = 50; dt = 1/fs;
    drone_time_new = (0:N-1)' * dt;
    drone_x = drone_data.pos_x;
    payload_time = payload_data.timestamp;
    payload_x = payload_data.pos_x;
    payload_x_nearest = zeros(N,1);
    for j = 1:N
        [~, idx] = min(abs(payload_time - drone_time_new(j)));
        payload_x_nearest(j) = payload_x(idx);
    end
    diff_x = drone_x - payload_x_nearest;
    all_diff_x{i} = diff_x;
    all_diff_x_min = min(all_diff_x_min, min(diff_x));
    all_diff_x_max = max(all_diff_x_max, max(diff_x));
    all_time_max = max(all_time_max, max(drone_time_new));
end

% 最大範囲から目盛り間隔を決定（例：5分割程度）
tick_num = 5;
tick_interval = (all_diff_x_max - all_diff_x_min) / tick_num;
ytick_vec = all_diff_x_min:tick_interval:all_diff_x_max;

for i = 1:size(file_gain_list_sorted,1)
    subplot(2,6,i);
    csv_file = file_gain_list_sorted{i,1};
    gain = file_gain_list_sorted{i,2};
    N = length(all_diff_x{i});
    fs = 50; dt = 1/fs;
    drone_time_new = (0:N-1)' * dt;
    diff_x = all_diff_x{i};
    plot(drone_time_new, diff_x, '-', 'LineWidth', 1.5, 'Color', color_list(i,:));
    hold on;
    grid on;
    xlabel('Time [s]');
    ylabel('X Pos Diff [m]');
    yticks(ytick_vec);
    xlim([0, all_time_max]);

    % ピーク検出（絶対値で局所最大）
    [pks, locs] = findpeaks(abs(diff_x), 'MinPeakProminence', 0.01);
    peak_times = drone_time_new(locs);
    % ピーク値を赤丸で重ねて描画
    plot(peak_times, diff_x(locs), 'ro', 'MarkerSize', 7, 'LineWidth', 1.5);

    % 対数減衰率の計算（ピーク値の対数を線形近似）
    if length(pks) >= 2
        log_pks = log(pks);
        p = polyfit(peak_times, log_pks, 1); % 1次近似
        damping_rate = -p(1); % 減衰率（正値）
        title(sprintf('Gain=%.2f\nDamping=%.4f\n%s', gain, damping_rate, csv_file));
    else
        title(sprintf('Gain=%.2f\nDamping=--\n%s', gain, csv_file));
    end
    end
sgtitle('Drone-Payload X Position Difference (All Files, Sorted by Gain)');
