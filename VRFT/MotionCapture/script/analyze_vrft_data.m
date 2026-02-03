% VRFT CSV Data Analysis Script
% 2つのCSVファイルからrigid_body_id=1のpos_xデータを抽出し、
% 速度を計算してresultに保存し、PDFでグラフを出力する

clear; close all; clc;
%% オフセット設定（最初の縦線位置・0.5秒後の線のオフセット）
vertical_line_offset = 0.1; % 開始点のオフセット（秒）
vertical_line_offset_2 = 2.0; % 2.0秒後の線の追加オフセット（秒）

%% パス設定
csv_folder = '..\CSV\';
result_folder = '..\result\';

% resultフォルダが存在しない場合は作成
if ~exist(result_folder, 'dir')
    mkdir(result_folder);
end

%% ファイルリスト
csv_files = {
    'record_20260203_135931.csv', ...
    'record_20260203_140036.csv'
};

%% サンプリング周波数
fs = 50; % Hz
dt = 1/fs; % サンプリング時間 = 0.02秒

%% 各ファイルを処理
for file_idx = 1:length(csv_files)
    csv_file = csv_files{file_idx};
    fprintf('Processing: %s\n', csv_file);
    
    %% CSVファイルの読み込み
    full_path = fullfile(csv_folder, csv_file);
    data = readtable(full_path);

    %% 全行から0行を検出
    data_mat = table2array(data);
    zero_all_idx = find(all(data_mat == 0, 2));
    % 連続区間ごとに分割
    zero_ranges = [];
    if ~isempty(zero_all_idx)
        d = diff(zero_all_idx);
        breaks = find(d > 1);
        zero_starts = [zero_all_idx(1); zero_all_idx(breaks+1)];
        zero_ends = [zero_all_idx(breaks); zero_all_idx(end)];
        zero_ranges = [zero_starts zero_ends];
    end

    %% rigid_body_id=1のデータのみを抽出（グラフ・速度計算用）
    id1_data = data(data.rigid_body_id == 1, :);
    n_samples = height(id1_data);
    fprintf('  Found %d samples with rigid_body_id=1\n', n_samples);

    %% Z軸のみ抽出
    pos_z = id1_data.pos_z;

    %% タイムスタンプを生成 (0から開始、50Hz)
    timestamp = (0:n_samples-1)' * dt;

    %% Z軸速度を計算（中心差分法、端点は前進/後退差分）
    vel_z = zeros(n_samples, 1);
    if n_samples > 1
        vel_z(1) = (pos_z(2) - pos_z(1)) / dt;
        for i = 2:n_samples-1
            vel_z(i) = (pos_z(i+1) - pos_z(i-1)) / (2*dt);
        end
        vel_z(n_samples) = (pos_z(n_samples) - pos_z(n_samples-1)) / dt;
    end

    %% 0区間を除いたデータを作成
    valid_mask = ~(pos_z == 0 & vel_z == 0);
    timestamp_valid = timestamp(valid_mask);
    pos_z_valid = pos_z(valid_mask);
    vel_z_valid = vel_z(valid_mask);

    %% 結果テーブルを作成
    result_table = table(timestamp, pos_z, vel_z, ...
        'VariableNames', {'Time_s', 'Position_Z_m', 'Velocity_Z_m_s'});
    
    %% 結果を保存
    [~, base_name, ~] = fileparts(csv_file);
    result_file = fullfile(result_folder, [base_name '_analyzed.csv']);
    writetable(result_table, result_file);
    fprintf('  Saved result to: %s\n', result_file);
    
    %% グラフ作成
    figure('Position', [100, 100, 1200, 500]);
    
    % Z軸のみの位置・速度プロット
    subplot(2, 1, 1);
    plot(timestamp_valid, pos_z_valid, 'r-', 'LineWidth', 1.5); hold on;
    id1_idx_all = find(data.rigid_body_id == 1);
    % ずらした縦線のみ表示
    if size(zero_ranges,1) >= 1
        idx_before = find(id1_idx_all < zero_ranges(1,1), 1, 'last');
        if ~isempty(idx_before)
            tline = timestamp(idx_before) + vertical_line_offset;
            xline(tline, 'k--', 'LineWidth', 1.8);
            tline_offset = tline + vertical_line_offset_2;
            if tline_offset <= timestamp(end)
                xline(tline_offset, 'k:', 'LineWidth', 1.8);
            end
        end
    end
    hold off;
    grid on;
    xlabel('Time [s]', 'FontSize', 12);
    ylabel('Position Z [m]', 'FontSize', 12);
    title(sprintf('Position Z - %s', base_name), 'FontSize', 14, 'Interpreter', 'none');

    subplot(2, 1, 2);
    plot(timestamp_valid, vel_z_valid, 'r-', 'LineWidth', 1.5); hold on;
    if size(zero_ranges,1) >= 1
        idx_before = find(id1_idx_all < zero_ranges(1,1), 1, 'last');
        if ~isempty(idx_before)
            tline = timestamp(idx_before) + vertical_line_offset;
            xline(tline, 'k--', 'LineWidth', 1.8);
            tline_offset = tline + vertical_line_offset_2;
            if tline_offset <= timestamp(end)
                xline(tline_offset, 'k:', 'LineWidth', 1.8);
            end
        end
    end
    hold off;
    grid on;
    xlabel('Time [s]', 'FontSize', 12);
    ylabel('Velocity Z [m/s]', 'FontSize', 12);
    title(sprintf('Velocity Z - %s', base_name), 'FontSize', 14, 'Interpreter', 'none');
    
    %% PDFとして保存（座標軸ツールバーを非表示にして最新状態で保存）
    ax = findall(gcf, 'type', 'axes');
    for a = 1:length(ax)
        axtoolbar(ax(a), {});
    end
    drawnow;
    pdf_file = fullfile(result_folder, [base_name '_plot.pdf']);
    set(gcf, 'PaperPositionMode', 'auto');
    print(gcf, pdf_file, '-dpdf', '-fillpage');
    fprintf('  Saved plot to: %s\n', pdf_file);
    
    fprintf('  Completed processing %s\n\n', csv_file);
end

fprintf('All files processed successfully!\n');
