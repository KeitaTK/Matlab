% RLS 振れ制御 Gain別 pos_x 比較プロット（指定グラフのみ表示）
% 2026/01/16 新規作成
% 指定インデックス（1,4,6,7,9,11番目）のみ表示

clear all; close all; clc;

% ファイルリストとGain値（写真・リストより）
file_gain_list = {
script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, '..', 'data', 'motion_capture');
    fullfile(data_dir, 'record_20260114_173422.csv'), 0;    % 1
    fullfile(data_dir, 'record_20260114_173622.csv'), 0;    % 2
    fullfile(data_dir, 'record_20260114_175531.csv'), 0.04; % 3
    fullfile(data_dir, 'record_20260114_175653.csv'), 0.04; % 4
    fullfile(data_dir, 'record_20260114_181118.csv'), 0.08; % 5
    fullfile(data_dir, 'record_20260114_181237.csv'), 0.08; % 6
    fullfile(data_dir, 'record_20260114_183840.csv'), 0.12; % 7
    fullfile(data_dir, 'record_20260114_184006.csv'), 0.12; % 8
    fullfile(data_dir, 'record_20260114_184410.csv'), 0.16; % 9
    fullfile(data_dir, 'record_20260114_184516.csv'), 0.16; % 10
    fullfile(data_dir, 'record_20260114_185540.csv'), 0.20; % 11
    fullfile(data_dir, 'record_20260114_185645.csv'), 0.20; % 12
};

color_list = lines(size(file_gain_list,1));

% 表示するインデックス
show_idx = [1,4,6,7,9,11];

% --- x軸のみの差分を計算・保存 ---
drone_field = 'pos_x';
all_diff = cell(size(file_gain_list,1), 1);
all_time_vec = cell(size(file_gain_list,1), 1);
all_center = zeros(size(file_gain_list,1), 1);
all_damping_rate = nan(size(file_gain_list,1), 1);
all_min = inf;
all_max = -inf;
all_time_min = inf;
all_time_max = -inf;

fs = 50;
dt = 1/fs;
decay_end_time = 5; % 減衰率計算の終了時刻 [秒]
offset_before_zero = -1; % 0検出点の何秒前をt=0にするか [秒]

for i = 1:size(file_gain_list,1)
    csv_file = file_gain_list{i,1};
    data = readtable(csv_file);
    drone_rows = data.rigid_body_id == 1;
    payload_rows = data.rigid_body_id == 2;
    drone_data = data(drone_rows, :);
    payload_data = data(payload_rows, :);
    N = height(drone_data);
    zero_mask_all = (data.pos_x == 0) & (data.pos_y == 0) & (data.pos_z == 0) & ...
                   (data.quat_x == 0) & (data.quat_y == 0) & (data.quat_z == 0) & (data.quat_w == 0);
    zero_idx_all = find(zero_mask_all, 1, 'first');
    if isempty(zero_idx_all)
        zero_idx_all = 1;
    end
    if mod(zero_idx_all,2)==1
        zero_idx = (zero_idx_all+1)/2;
    else
        zero_idx = zero_idx_all/2;
    end
    zero_idx = min(max(zero_idx,1),N);
    time_vec = ((0:N-1)' - (zero_idx-1) - round(offset_before_zero/dt)) * dt;
    drone_val = drone_data.(drone_field);
    payload_val = payload_data.(drone_field);
    diff_val = drone_val - payload_val;
    post_zero_mask = time_vec >= 0;
    if sum(post_zero_mask) > 0
        center_val = mean(diff_val(post_zero_mask));
    else
        center_val = mean(diff_val);
    end
    all_diff{i} = diff_val;
    all_time_vec{i} = time_vec;
    all_center(i) = center_val;
    all_min = min(all_min, min(diff_val));
    all_max = max(all_max, max(diff_val));
    all_time_min = min(all_time_min, min(time_vec));
    all_time_max = max(all_time_max, max(time_vec));
    % 減衰率計算はコメントアウト
    % decay_mask = (time_vec >= 0) & (time_vec <= decay_end_time);
    % if sum(decay_mask) > 10
    %     time_decay = time_vec(decay_mask);
    %     diff_decay = diff_val(decay_mask);
    %     [pks, locs] = findpeaks(abs(diff_decay - center_val), 'MinPeakProminence', 0.005);
    %     if length(pks) >= 2
    %         log_pks = log(pks);
    %         peak_times = time_decay(locs);
    %         p = polyfit(peak_times, log_pks, 1);
    %         all_damping_rate(i) = -p(1);
    %     end
    % end
end

tick_num = 5;
tick_interval = (all_max - all_min) / tick_num;
ytick_vec = all_min:tick_interval:all_max;

figure('Position', [100, 100, 1200, 600], 'Color', 'white');
for idx = 1:length(show_idx)
    i = show_idx(idx);
    subplot(length(show_idx),1,idx);
    gain = file_gain_list{i,2};
    time_vec = all_time_vec{i};
    diff_val = all_diff{i};
    center_val = all_center(i);
    plot(time_vec, diff_val, '-', 'LineWidth', 1.5, 'Color', color_list(i,:));
    hold on;
    plot([0, 0], [all_min, all_max], 'r-', 'LineWidth', 2);
    plot([all_time_min, all_time_max], [center_val, center_val], 'g--', 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('X Pos Diff [m]');
    ylim([all_min, all_max]);
    yticks(ytick_vec);
    xlim([all_time_min, all_time_max]);
    title(sprintf('Gain=%.2f\nCenter=%.4f m', gain, center_val), 'FontSize', 10);
end
sgtitle('Selected Drone-Payload X Position Difference (Gain別, 50Hz)');

% 保存
[~, script_name, ~] = fileparts(mfilename('fullpath'));
output_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'figures', script_name);
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
file_pattern = fullfile(output_dir, 'plot_gain_compare_selected_*.png');
files = dir(file_pattern);
next_num = numel(files) + 1;
output_file = fullfile(output_dir, sprintf('plot_gain_compare_selected_%03d.png', next_num));
saveas(gcf, output_file);

fprintf('図を保存しました: %s\n', output_file);
