% RLS 振れ制御 Gain別 pos_x 比較プロット
% 2026/01/14 新規作成
% ファイルとGain値の対応は写真・リストより

clear all; close all; clc;

% ファイルリストとGain値（写真・リストより）
script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, 'csv');
file_gain_list = {
    fullfile(data_dir, 'record_20260114_173422.csv'), 0;
    fullfile(data_dir, 'record_20260114_173622.csv'), 0;
    fullfile(data_dir, 'record_20260114_175531.csv'), 0.04;
    fullfile(data_dir, 'record_20260114_175653.csv'), 0.04;
    fullfile(data_dir, 'record_20260114_181118.csv'), 0.08;
    fullfile(data_dir, 'record_20260114_181237.csv'), 0.08;
    fullfile(data_dir, 'record_20260114_183840.csv'), 0.12;
    fullfile(data_dir, 'record_20260114_184006.csv'), 0.12;
    fullfile(data_dir, 'record_20260114_184410.csv'), 0.16;
    fullfile(data_dir, 'record_20260114_184516.csv'), 0.16;
    fullfile(data_dir, 'record_20260114_185540.csv'), 0.20;
    fullfile(data_dir, 'record_20260114_185645.csv'), 0.20;
};

color_list = lines(size(file_gain_list,1));

% --- x軸のみの差分を計算・保存 ---
% サンプリング周波数: 50Hz
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
    
    % 元のデータは交互に並んでいる：行1,3,5,... = ID1, 行2,4,6,... = ID2
    drone_rows = data.rigid_body_id == 1;
    payload_rows = data.rigid_body_id == 2;
    
    drone_data = data(drone_rows, :);
    payload_data = data(payload_rows, :);
    
    N = height(drone_data);
    
    % 全ての値が0の行をCSV全体から検出
    zero_mask_all = (data.pos_x == 0) & (data.pos_y == 0) & (data.pos_z == 0) & ...
                   (data.quat_x == 0) & (data.quat_y == 0) & (data.quat_z == 0) & (data.quat_w == 0);
    zero_idx_all = find(zero_mask_all, 1, 'first');
    if isempty(zero_idx_all)
        zero_idx_all = 1;
    end

    % ドローンデータのt=0インデックスを計算
    if mod(zero_idx_all,2)==1
        zero_idx = (zero_idx_all+1)/2;
    else
        zero_idx = zero_idx_all/2;
    end
    zero_idx = min(max(zero_idx,1),N);

    % 時刻ベクトル：0検出点のoffset_before_zero秒前をt=0にする
    time_vec = ((0:N-1)' - (zero_idx-1) - round(offset_before_zero/dt)) * dt;
    
    drone_val = drone_data.(drone_field);
    payload_val = payload_data.(drone_field);
    
    % 差分計算
    diff_val = drone_val - payload_val;
    
    % t>=0の部分で振動中心（平均値）を計算
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
    
    % t=0からdecay_end_timeまでの区間で減衰率計算
    decay_mask = (time_vec >= 0) & (time_vec <= decay_end_time);
    if sum(decay_mask) > 10
        time_decay = time_vec(decay_mask);
        diff_decay = diff_val(decay_mask);
        [pks, locs] = findpeaks(abs(diff_decay - center_val), 'MinPeakProminence', 0.005);
        if length(pks) >= 2
            log_pks = log(pks);
            peak_times = time_decay(locs);
            p = polyfit(peak_times, log_pks, 1);
            all_damping_rate(i) = -p(1);
        end
    end
end

% Y軸設定
tick_num = 5;
tick_interval = (all_max - all_min) / tick_num;
ytick_vec = all_min:tick_interval:all_max;

% x軸のみのプロット
figure('Position', [100, 100, 1600, 600]);
gains = zeros(size(file_gain_list,1), 1);

for i = 1:size(file_gain_list,1)
    subplot(4,4,i);
    gain = file_gain_list{i,2};
    gains(i) = gain;
    time_vec = all_time_vec{i};
    diff_val = all_diff{i};
    center_val = all_center(i);
    damping_rate = all_damping_rate(i);
    
    plot(time_vec, diff_val, '-', 'LineWidth', 1.5, 'Color', color_list(i,:));
    hold on;
    
    % t=0の位置に赤い縦線を引く
    plot([0, 0], [all_min, all_max], 'r-', 'LineWidth', 2);
    
    % 振動中心を緑の破線で表示
    plot([all_time_min, all_time_max], [center_val, center_val], 'g--', 'LineWidth', 1.5);
    
    grid on;
    xlabel('Time [s]');
    ylabel('X Pos Diff [m]');
    ylim([all_min, all_max]);
    yticks(ytick_vec);
    xlim([all_time_min, all_time_max]);

    % t=0からdecay_end_timeまでの区間で減衰曲線を表示
    decay_mask = (time_vec >= 0) & (time_vec <= decay_end_time);
    if sum(decay_mask) > 10
        time_decay = time_vec(decay_mask);
        diff_decay = diff_val(decay_mask);
        [pks, locs] = findpeaks(abs(diff_decay - center_val), 'MinPeakProminence', 0.005);
        
        % 減衰率が有効な場合、指数減衰曲線を表示
        if ~isnan(damping_rate) && damping_rate > 0
            % 最初のピークの大きさから初期振幅を計算
            if ~isempty(pks)
                A0 = pks(1);
                time_fit = linspace(0, decay_end_time, 200);
                % 指数減衰：A(t) = A0 * exp(-damping_rate * t)
                envelope_pos = center_val + A0 * exp(-damping_rate * time_fit);
                envelope_neg = center_val - A0 * exp(-damping_rate * time_fit);
                plot(time_fit, envelope_pos, 'r-', 'LineWidth', 2, 'DisplayName', 'Envelope');
                plot(time_fit, envelope_neg, 'r-', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
        end
    end
    
    if ~isnan(damping_rate)
        title(sprintf('Gain=%.2f, Damp=%.4f\nCenter=%.4f m', gain, damping_rate, center_val), 'FontSize', 8);
    else
        title(sprintf('Gain=%.2f\nCenter=%.4f m', gain, center_val), 'FontSize', 8);
    end
end
sgtitle('Drone-Payload X Position Difference (Gain別, 50Hz)');


% プログラム名ごとにfigures内にサブフォルダを作成
[~, script_name, ~] = fileparts(mfilename('fullpath'));
output_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'figures', script_name);
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
file_pattern = fullfile(output_dir, 'plot_gain_compare_X_*.png');
files = dir(file_pattern);
next_num = numel(files) + 1;
output_file = fullfile(output_dir, sprintf('plot_gain_compare_X_%03d.png', next_num));
saveas(gcf, output_file);



% 減衰率 vs ゲインのプロット（ダンピングのみ＋同じゲインの平均も表示）
figure('Position', [200, 200, 900, 400]);
hold on;
% 個々の点
plot(gains, all_damping_rate, 'o', 'MarkerSize', 8, 'LineWidth', 1.5, 'Color', [0.2 0.4 0.8], 'DisplayName', 'Each Trial');


% 同じゲインごとに平均を計算し、線でつなぐ
[unique_gains, ~, idxu] = unique(gains);
mean_damping = accumarray(idxu, all_damping_rate, [], @mean);
plot(unique_gains, mean_damping, '-s', 'MarkerSize', 12, 'MarkerFaceColor', [1 0.4 0.2], 'MarkerEdgeColor', [0.8 0.2 0], 'LineWidth', 2, 'DisplayName', 'Mean per Gain');

grid on;
xlabel('Gain');
ylabel('Damping Rate');
title('Damping Rate vs Gain (Mean per Gain Highlighted)');
legend('show','Location','best');

% このまとめグラフもPNGで保存
file_pattern_gain = fullfile(output_dir, 'plot_damping_gain_*.png');
files_gain = dir(file_pattern_gain);
next_num_gain = numel(files_gain) + 1;
output_file_gain = fullfile(output_dir, sprintf('plot_damping_gain_%03d.png', next_num_gain));
saveas(gcf, output_file_gain);


