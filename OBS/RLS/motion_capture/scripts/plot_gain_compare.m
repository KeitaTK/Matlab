% RLS 振れ制御 Gain別 pos_x 比較プロット
% 2026/01/14 新規作成
% ファイルとGain値の対応は写真・リストより

clear all; close all; clc;

% ファイルリストとGain値（写真・リストより）
file_gain_list = {
    'C:/Users/taki/Local/local/Matlab/OBS/RLS/motion_capture/csv/record_20260114_173422.csv', 0;
    'C:/Users/taki/Local/local/Matlab/OBS/RLS/motion_capture/csv/record_20260114_173622.csv', 0;
    'C:/Users/taki/Local/local/Matlab/OBS/RLS/motion_capture/csv/record_20260114_175531.csv', 0.04;
    'C:/Users/taki/Local/local/Matlab/OBS/RLS/motion_capture/csv/record_20260114_175653.csv', 0.04;
    'C:/Users/taki/Local/local/Matlab/OBS/RLS/motion_capture/csv/record_20260114_181118.csv', 0.08;
    'C:/Users/taki/Local/local/Matlab/OBS/RLS/motion_capture/csv/record_20260114_181237.csv', 0.08;
    'C:/Users/taki/Local/local/Matlab/OBS/RLS/motion_capture/csv/record_20260114_183840.csv', 0.12;
    'C:/Users/taki/Local/local/Matlab/OBS/RLS/motion_capture/csv/record_20260114_184006.csv', 0.12;
    'C:/Users/taki/Local/local/Matlab/OBS/RLS/motion_capture/csv/record_20260114_184410.csv', 0.16;
    'C:/Users/taki/Local/local/Matlab/OBS/RLS/motion_capture/csv/record_20260114_184516.csv', 0.16;
    'C:/Users/taki/Local/local/Matlab/OBS/RLS/motion_capture/csv/record_20260114_185540.csv', 0.16;
    'C:/Users/taki/Local/local/Matlab/OBS/RLS/motion_capture/csv/record_20260114_185645.csv', 0.20;
};

color_list = lines(size(file_gain_list,1));
figure('Position', [100, 100, 1600, 600]);


% --- 3軸分の差分を計算・保存 ---
% 注：タイムスタンプは-1と0のみなので、行番号で時刻対応させる
% サンプリング周波数: 50Hz
axes_names = {'X', 'Y', 'Z'};
drone_fields = {'pos_x', 'pos_y', 'pos_z'};
all_diff = cell(3, size(file_gain_list,1)); % {軸, ファイル}
all_time_vec = cell(size(file_gain_list,1), 1); % 時刻ベクトル（0基準）
all_zero_time = zeros(size(file_gain_list,1), 1); % 0検出時刻
all_center = zeros(3, size(file_gain_list,1)); % 振動中心
all_min = inf(1,3);
all_max = -inf(1,3);
all_time_min = inf;
all_time_max = -inf;

fs = 50; % サンプリング周波数 50Hz
dt = 1/fs; % 0.02秒

for axis_idx = 1:3
    for i = 1:size(file_gain_list,1)
        csv_file = file_gain_list{i,1};
        data = readtable(csv_file);
        
        % 元のデータは交互に並んでいる：行1,3,5,... = ID1, 行2,4,6,... = ID2
        drone_rows = data.rigid_body_id == 1;
        payload_rows = data.rigid_body_id == 2;
        
        drone_data = data(drone_rows, :);
        payload_data = data(payload_rows, :);
        
        N = height(drone_data);
        
        % 全ての値が0の行を検出（ドローンデータで）
        zero_mask = (drone_data.pos_x == 0) & (drone_data.pos_y == 0) & (drone_data.pos_z == 0) & ...
                    (drone_data.quat_x == 0) & (drone_data.quat_y == 0) & (drone_data.quat_z == 0) & (drone_data.quat_w == 0);
        zero_idx = find(zero_mask, 1, 'first'); % 最初の0行を検出
        
        if isempty(zero_idx)
            zero_idx = 1; % 見つからない場合は1を使用
        end
        
        % 時刻ベクトル：0行を基準(t=0)にする
        time_vec = ((0:N-1)' - (zero_idx-1)) * dt;
        
        drone_val = drone_data.(drone_fields{axis_idx});
        payload_val = payload_data.(drone_fields{axis_idx});
        
        % 行番号順で直接対応（タイムスタンプではなく）
        diff_val = drone_val - payload_val;
        
        % t>=0の部分で振動中心（平均値）を計算
        post_zero_mask = time_vec >= 0;
        if sum(post_zero_mask) > 0
            center_val = mean(diff_val(post_zero_mask));
        else
            center_val = mean(diff_val);
        end
        
        all_diff{axis_idx, i} = diff_val;
        all_time_vec{i} = time_vec;
        all_zero_time(i) = 0; % 常に0
        all_center(axis_idx, i) = center_val;
        all_min(axis_idx) = min(all_min(axis_idx), min(diff_val));
        all_max(axis_idx) = max(all_max(axis_idx), max(diff_val));
        all_time_min = min(all_time_min, min(time_vec));
        all_time_max = max(all_time_max, max(time_vec));
    end
end

tick_num = 5;
ytick_vecs = cell(1,3);
for axis_idx = 1:3
    tick_interval = (all_max(axis_idx) - all_min(axis_idx)) / tick_num;
    ytick_vecs{axis_idx} = all_min(axis_idx):tick_interval:all_max(axis_idx);
end

for axis_idx = 1:3
    figure('Position', [100, 100, 1600, 600]);
    for i = 1:size(file_gain_list,1)
        subplot(4,4,i);
        csv_file = file_gain_list{i,1};
        gain = file_gain_list{i,2};
        time_vec = all_time_vec{i};
        diff_val = all_diff{axis_idx, i};
        center_val = all_center(axis_idx, i);
        
        plot(time_vec, diff_val, '-', 'LineWidth', 1.5, 'Color', color_list(i,:));
        hold on;
        
        % t=0の位置に赤い縦線を引く
        plot([0, 0], [all_min(axis_idx), all_max(axis_idx)], 'r-', 'LineWidth', 2);
        
        % 振動中心を緑の破線で表示
        plot([all_time_min, all_time_max], [center_val, center_val], 'g--', 'LineWidth', 1.5);
        
        grid on;
        xlabel('Time [s]');
        ylabel([axes_names{axis_idx} ' Pos Diff [m]']);
        ylim([all_min(axis_idx), all_max(axis_idx)]);
        yticks(ytick_vecs{axis_idx});
        xlim([all_time_min, all_time_max]);

        % t>=0の部分でピーク検出と減衰率計算
        post_zero_mask = time_vec >= 0;
        if sum(post_zero_mask) > 10
            time_post = time_vec(post_zero_mask);
            diff_post = diff_val(post_zero_mask);
            
            [pks, locs] = findpeaks(abs(diff_post - center_val), 'MinPeakProminence', 0.005);
            if ~isempty(locs)
                peak_times = time_post(locs);
                plot(peak_times, diff_post(locs), 'ro', 'MarkerSize', 7, 'LineWidth', 1.5);
                
                if length(pks) >= 2
                    log_pks = log(pks);
                    p = polyfit(peak_times, log_pks, 1);
                    damping_rate = -p(1);
                    title(sprintf('Gain=%.2f, Damp=%.4f\nCenter=%.4f m', gain, damping_rate, center_val), 'FontSize', 8);
                else
                    title(sprintf('Gain=%.2f\nCenter=%.4f m', gain, center_val), 'FontSize', 8);
                end
            else
                title(sprintf('Gain=%.2f\nCenter=%.4f m', gain, center_val), 'FontSize', 8);
            end
        else
            title(sprintf('Gain=%.2f\nCenter=%.4f m', gain, center_val), 'FontSize', 8);
        end
    end
    sgtitle(['Drone-Payload ' axes_names{axis_idx} ' Position Difference (Gain別, 50Hz)']);

    % PNGとして連番保存
    output_dir = 'C:/Users/taki/Local/local/Matlab/OBS/RLS/motion_capture/figures';
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    file_pattern = fullfile(output_dir, ['plot_gain_compare_' axes_names{axis_idx} '_*.png']);
    files = dir(file_pattern);
    next_num = numel(files) + 1;
    output_file = fullfile(output_dir, sprintf('plot_gain_compare_%s_%03d.png', axes_names{axis_idx}, next_num));
    saveas(gcf, output_file);
end
