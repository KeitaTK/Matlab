% RLS 振れ制御 Gain 0 vs 0.20 重ね合わせ比較
% 2026/01/15 新規作成
% gain=0の15秒時点とgain=0.20の7秒時点を同じ時刻に揃える

clear all; close all; clc;

% --- 設定: 評価・表示範囲を変数で指定 ---
energy_eval_tmin = 0;    % エネルギー減衰率評価の開始時刻 [s]
energy_eval_tmax = 10;   % エネルギー減衰率評価の終了時刻 [s]
graph_tmin = 0;          % グラフ表示の開始時刻 [s]
graph_tmax = 30;         % グラフ表示の終了時刻 [s]

% 比較する2つのファイル（相対パス）
script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, '..', 'data', 'motion_capture');
file1 = fullfile(data_dir, 'record_20260114_173622.csv');
gain1 = 0;
start_time1 = 4.16; % gain=0のこの時刻をt=0にする

file2 = fullfile(data_dir, 'record_20260114_185540.csv');
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

% 全ての値が0の行を検出
zero_mask_all1 = (data1.pos_x == 0) & (data1.pos_y == 0) & (data1.pos_z == 0) & ...
               (data1.quat_x == 0) & (data1.quat_y == 0) & (data1.quat_z == 0) & (data1.quat_w == 0);
zero_idx_all1 = find(zero_mask_all1, 1, 'first');
if isempty(zero_idx_all1)
    zero_idx_all1 = 1;
end

% ドローンデータのt=0インデックスを計算
if mod(zero_idx_all1,2)==1
    zero_idx1 = (zero_idx_all1+1)/2;
else
    zero_idx1 = zero_idx_all1/2;
end
zero_idx1 = min(max(zero_idx1,1),N1);

% 時刻ベクトル：0検出点のoffset_before_zero秒前をt=0にする
time_vec1 = ((0:N1-1)' - (zero_idx1-1) - round(offset_before_zero/dt)) * dt;

drone_val1 = drone_data1.(drone_field);
payload_val1 = payload_data1.(drone_field);

% 差分計算
diff_val1 = drone_val1 - payload_val1;

% t>=0の部分で振動中心を計算
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

% 全ての値が0の行を検出
zero_mask_all2 = (data2.pos_x == 0) & (data2.pos_y == 0) & (data2.pos_z == 0) & ...
               (data2.quat_x == 0) & (data2.quat_y == 0) & (data2.quat_z == 0) & (data2.quat_w == 0);
zero_idx_all2 = find(zero_mask_all2, 1, 'first');
if isempty(zero_idx_all2)
    zero_idx_all2 = 1;
end

% ドローンデータのt=0インデックスを計算
if mod(zero_idx_all2,2)==1
    zero_idx2 = (zero_idx_all2+1)/2;
else
    zero_idx2 = zero_idx_all2/2;
end
zero_idx2 = min(max(zero_idx2,1),N2);

% 時刻ベクトル：0検出点のoffset_before_zero秒前をt=0にする
time_vec2_original = ((0:N2-1)' - (zero_idx2-1) - round(offset_before_zero/dt)) * dt;

drone_val2 = drone_data2.(drone_field);
payload_val2 = payload_data2.(drone_field);

% 差分計算
diff_val2 = drone_val2 - payload_val2;

% t>=0の部分で振動中心を計算
post_zero_mask2 = time_vec2_original >= 0;
if sum(post_zero_mask2) > 0
    center_val2 = mean(diff_val2(post_zero_mask2));
else
    center_val2 = mean(diff_val2);
end

% --- 時刻の調整 ---
% 各データの開始点を0に揃える
time_vec1_adjusted = time_vec1 - start_time1;
time_vec2_adjusted = time_vec2_original - start_time2;

% --- 開始点以降のデータのみ表示 ---
% gain=0: t>=0かつt<=30のデータ
valid_mask1 = (time_vec1_adjusted >= graph_tmin) & (time_vec1_adjusted <= graph_tmax);
time_vec1_plot = time_vec1_adjusted(valid_mask1);
diff_val1_plot = (diff_val1(valid_mask1) - center_val1);  % 中心値を0に揃える

% gain=0.20: t>=0かつt<=30のデータ
valid_mask2 = (time_vec2_adjusted >= graph_tmin) & (time_vec2_adjusted <= graph_tmax);
time_vec2_plot = time_vec2_adjusted(valid_mask2);
diff_val2_plot = (diff_val2(valid_mask2) - center_val2);  % 中心値を0に揃える

% --- エネルギー計算（中心値を引いた後） ---
energy_vec1 = (diff_val1_plot).^2;
energy_vec2 = (diff_val2_plot).^2;

energy_mask1 = (time_vec1_plot >= energy_eval_tmin) & (time_vec1_plot <= energy_eval_tmax);
energy_vec1_10s = energy_vec1(energy_mask1);
time_vec1_plot_10s = time_vec1_plot(energy_mask1);

energy_mask2 = (time_vec2_plot >= energy_eval_tmin) & (time_vec2_plot <= energy_eval_tmax);
energy_vec2_10s = energy_vec2(energy_mask2);
time_vec2_plot_10s = time_vec2_plot(energy_mask2);

% --- プロット ---
figure('Position', [100, 100, 1200, 800]);

% 位置差分の重ね合わせ
subplot(2,1,1);
plot(time_vec1_plot, diff_val1_plot, '-', 'LineWidth', 2, 'Color', [0, 0.4470, 0.7410], 'DisplayName', sprintf('Gain=%.2f', gain1));
hold on;
plot(time_vec2_plot, diff_val2_plot, '-', 'LineWidth', 2, 'Color', [0.8500, 0.3250, 0.0980], 'DisplayName', sprintf('Gain=%.2f', gain2));

% 振動中心（0）を表示
ymin = min([min(diff_val1_plot), min(diff_val2_plot)]);
ymax = max([max(diff_val1_plot), max(diff_val2_plot)]);
xmin = min([min(time_vec1_plot), min(time_vec2_plot)]);
xmax = max([max(time_vec1_plot), max(time_vec2_plot)]);

% plot([0, 0], [ymin, ymax], 'k--', 'LineWidth', 2, 'DisplayName', 'Start point (t=0)');

grid on;
xlabel('Time [s]', 'FontSize', 12);
ylabel('\Deltax(t) [m]', 'FontSize', 12);
% title(sprintf('Drone-Payload X Position Difference Comparison\n(Gain %.2f starts at %.2fs, Gain %.2f starts at %.2fs)', gain1, start_time1, gain2, start_time2), 'FontSize', 14);
legend('Location', 'best', 'FontSize', 14);
ylim([ymin, ymax]);
xlim([xmin, xmax]);

% (a)を左上やや右に
text(graph_tmin + 0.10*(graph_tmax-graph_tmin), ymax - 0.05*(ymax-ymin), '(a)', 'FontSize', 16, 'FontWeight', 'bold', 'VerticalAlignment', 'top');

% エネルギー比較
subplot(2,1,2);

plot(time_vec1_plot, energy_vec1, '-', 'LineWidth', 2, 'Color', [0, 0.4470, 0.7410], 'DisplayName', sprintf('Gain=%.2f', gain1));
hold on;
plot(time_vec2_plot, energy_vec2, '-', 'LineWidth', 2, 'Color', [0.8500, 0.3250, 0.0980], 'DisplayName', sprintf('Gain=%.2f', gain2));

xlabel('Time [s]', 'FontSize', 12);
ylabel('E(t) = [\Deltax(t)]^2 [m^2]', 'FontSize', 12);
grid on;
legend('Location', 'best', 'FontSize', 14);
ylim([0, max([max(energy_vec1), max(energy_vec2)])]);
xlim([graph_tmin, graph_tmax]);

% (b)を左上やや右に
text(graph_tmin + 0.10*(graph_tmax-graph_tmin), max([max(energy_vec1), max(energy_vec2)]) - 0.05*max([max(energy_vec1), max(energy_vec2)]), '(b)', 'FontSize', 16, 'FontWeight', 'bold', 'VerticalAlignment', 'top');


% --- エネルギーピークのみ抽出しpolyfit ---
% gain=0
energy_mask1 = (time_vec1_plot >= energy_eval_tmin) & (time_vec1_plot <= energy_eval_tmax);
energy_vec1_10s = energy_vec1(energy_mask1);
time_vec1_plot_10s = time_vec1_plot(energy_mask1);

% ピーク抽出
[pks1, locs1] = findpeaks(energy_vec1_10s, time_vec1_plot_10s);

if ~isempty(pks1) && max(pks1) > 0
    valid_idx1 = pks1 > max(pks1)*1e-6;
    t1 = locs1(valid_idx1);
    e1 = pks1(valid_idx1);
    log_e1 = log(e1);
    p1 = polyfit(t1, log_e1, 1);
    decay_rate1 = -p1(1);
else
    decay_rate1 = NaN;
end

% gain=0.20
energy_mask2 = (time_vec2_plot >= energy_eval_tmin) & (time_vec2_plot <= energy_eval_tmax);
energy_vec2_10s = energy_vec2(energy_mask2);
time_vec2_plot_10s = time_vec2_plot(energy_mask2);

[pks2, locs2] = findpeaks(energy_vec2_10s, time_vec2_plot_10s);

if ~isempty(pks2) && max(pks2) > 0
    valid_idx2 = pks2 > max(pks2)*1e-6;
    t2 = locs2(valid_idx2);
    e2 = pks2(valid_idx2);
    log_e2 = log(e2);
    p2 = polyfit(t2, log_e2, 1);
    decay_rate2 = -p2(1);
else
    decay_rate2 = NaN;
end

% --- ピーク点をグラフに表示 ---
subplot(2,1,2);
hold on;
plot(locs1, pks1, 'o', 'Color', [0, 0.4470, 0.7410], 'MarkerFaceColor', [0, 0.4470, 0.7410], 'DisplayName', 'Gain=0 Peak');
plot(locs2, pks2, 'o', 'Color', [0.8500, 0.3250, 0.0980], 'MarkerFaceColor', [0.8500, 0.3250, 0.0980], 'DisplayName', 'Gain=0.20 Peak');

% 保存
[~, script_name, ~] = fileparts(mfilename('fullpath'));
output_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'figures', script_name);
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
fprintf('\n=== エネルギー減衰率の比較 ===\n');
fprintf('Gain=%.2f: 減衰率 λ = %.6f [1/s]\n', gain1, decay_rate1);
fprintf('Gain=%.2f: 減衰率 λ = %.6f [1/s]\n', gain2, decay_rate2);
fprintf('減衰率の比 (Gain %.2f / Gain %.2f) = %.4f\n', gain2, gain1, decay_rate2/decay_rate1);
fprintf('減衰を1/e(約37%%)まで低下させるのにかかる時間:\n');
fprintf('  Gain=%.2f: τ = %.2f [s]\n', gain1, 1/decay_rate1);
fprintf('  Gain=%.2f: τ = %.2f [s]\n', gain2, 1/decay_rate2);
