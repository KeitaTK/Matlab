% RLS 振れ制御 Gain 0 vs 0.20 位置差分比較（エネルギー評価なし・上グラフのみ）
% 2026/01/16 新規作成
% gain=0の15秒時点とgain=0.20の7秒時点を同じ時刻に揃える

clear all; close all; clc;

% --- 設定: 評価・表示範囲を変数で指定 ---
graph_tmin = 0;          % グラフ表示の開始時刻 [s]
graph_tmax = 30;         % グラフ表示の終了時刻 [s]

% 比較する2つのファイル
file1 = 'C:/Users/keita/Documents/Local/Matlab/OBS/RLS/motion_capture/csv/record_20260114_173622.csv';
gain1 = 0;
start_time1 = 4.16; % gain=0のこの時刻をt=0にする

file2 = 'C:/Users/keita/Documents/Local/Matlab/OBS/RLS/motion_capture/csv/record_20260114_185540.csv';
gain2 = 0.20;
start_time2 = 0.08; % gain=0.20のこの時刻をt=0にする

drone_field = 'pos_x';
fs = 50;
dt = 1/fs;
offset_before_zero = 0; % 0検出点の何秒前をt=0にするか [秒]

% --- ファイル1の読み込みと処理 ---
data1 = readtable(file1);

drone_rows1 = data1.rigid_body_id == 1;
payload_rows1 = data1.rigid_body_id == 2;

drone_data1 = data1(drone_rows1, :);
payload_data1 = data1(payload_rows1, :);

N1 = height(drone_data1);

zero_mask_all1 = (data1.pos_x == 0) & (data1.pos_y == 0) & (data1.pos_z == 0) & ...
               (data1.quat_x == 0) & (data1.quat_y == 0) & (data1.quat_z == 0) & (data1.quat_w == 0);
zero_idx_all1 = find(zero_mask_all1, 1, 'first');
if isempty(zero_idx_all1)
    zero_idx_all1 = 1;
end

if mod(zero_idx_all1,2)==1
    zero_idx1 = (zero_idx_all1+1)/2;
else
    zero_idx1 = zero_idx_all1/2;
end
zero_idx1 = min(max(zero_idx1,1),N1);

time_vec1 = ((0:N1-1)' - (zero_idx1-1) - round(offset_before_zero/dt)) * dt;

drone_val1 = drone_data1.(drone_field);
payload_val1 = payload_data1.(drone_field);

diff_val1 = drone_val1 - payload_val1;

post_zero_mask1 = time_vec1 >= 0;
if sum(post_zero_mask1) > 0
    center_val1 = mean(diff_val1(post_zero_mask1));
else
    center_val1 = mean(diff_val1);
end

% --- ファイル2の読み込みと処理 ---
data2 = readtable(file2);

drone_rows2 = data2.rigid_body_id == 1;
payload_rows2 = data2.rigid_body_id == 2;

drone_data2 = data2(drone_rows2, :);
payload_data2 = data2(payload_rows2, :);

N2 = height(drone_data2);

zero_mask_all2 = (data2.pos_x == 0) & (data2.pos_y == 0) & (data2.pos_z == 0) & ...
               (data2.quat_x == 0) & (data2.quat_y == 0) & (data2.quat_z == 0) & (data2.quat_w == 0);
zero_idx_all2 = find(zero_mask_all2, 1, 'first');
if isempty(zero_idx_all2)
    zero_idx_all2 = 1;
end

if mod(zero_idx_all2,2)==1
    zero_idx2 = (zero_idx_all2+1)/2;
else
    zero_idx2 = zero_idx_all2/2;
end
zero_idx2 = min(max(zero_idx2,1),N2);

time_vec2_original = ((0:N2-1)' - (zero_idx2-1) - round(offset_before_zero/dt)) * dt;

drone_val2 = drone_data2.(drone_field);
payload_val2 = payload_data2.(drone_field);

diff_val2 = drone_val2 - payload_val2;

post_zero_mask2 = time_vec2_original >= 0;
if sum(post_zero_mask2) > 0
    center_val2 = mean(diff_val2(post_zero_mask2));
else
    center_val2 = mean(diff_val2);
end

% --- 時刻の調整 ---
time_vec1_adjusted = time_vec1 - start_time1;
time_vec2_adjusted = time_vec2_original - start_time2;

% --- 開始点以降のデータのみ表示 ---
valid_mask1 = (time_vec1_adjusted >= graph_tmin) & (time_vec1_adjusted <= graph_tmax);
time_vec1_plot = time_vec1_adjusted(valid_mask1);
diff_val1_plot = (diff_val1(valid_mask1) - center_val1);

valid_mask2 = (time_vec2_adjusted >= graph_tmin) & (time_vec2_adjusted <= graph_tmax);
time_vec2_plot = time_vec2_adjusted(valid_mask2);
diff_val2_plot = (diff_val2(valid_mask2) - center_val2);

% --- プロット（上グラフのみ） ---
figure('Position', [100, 100, 1200, 600], 'Color', 'white');
plot(time_vec1_plot, diff_val1_plot, '-', 'LineWidth', 2, 'Color', [0, 0.4470, 0.7410], 'DisplayName', sprintf('Gain=%.2f', gain1));
hold on;
plot(time_vec2_plot, diff_val2_plot, '-', 'LineWidth', 2, 'Color', [0.8500, 0.3250, 0.0980], 'DisplayName', sprintf('Gain=%.2f', gain2));

ymin = min([min(diff_val1_plot), min(diff_val2_plot)]);
ymax = max([max(diff_val1_plot), max(diff_val2_plot)]);
xmin = min([min(time_vec1_plot), min(time_vec2_plot)]);
xmax = max([max(time_vec1_plot), max(time_vec2_plot)]);

grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('\Deltax(t) [m]', 'FontSize', 12);
legend('Location', 'best', 'FontSize', 14);
ylim([ymin, ymax]);
xlim([xmin, xmax]);
% ...existing code...

% 保存
[~, script_name, ~] = fileparts(mfilename('fullpath'));
output_dir = fullfile('C:/Users/keita/Documents/Local/Matlab/OBS/RLS/motion_capture/figures', script_name);
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
file_pattern = fullfile(output_dir, 'overlay_comparison_*.png');
files = dir(file_pattern);
next_num = numel(files) + 1;
output_file = fullfile(output_dir, sprintf('overlay_comparison_%03d.png', next_num));
saveas(gcf, output_file);

fprintf('図を保存しました: %s\n', output_file);
fprintf('Gain=%.2f: 振動中心 = %.4f m\n', gain1, center_val1);
fprintf('Gain=%.2f: 振動中心 = %.4f m\n', gain2, center_val2);
