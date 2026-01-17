%% 外力データ可視化プログラム（10-18秒間、ローパスフィルタ適用）
clear; clc; close all;

% CSVファイル読み込み
% filename = '20250721_192836_EF.csv';  % 吊り荷なし
filename = '20250721_194808_EF.csv';  % 吊り荷あり
% Build data file path relative to this script
script_dir = fileparts(mfilename('fullpath'));
datafile = fullfile(script_dir, '..', 'data', filename);
opts = detectImportOptions(datafile);
opts.VariableNamesLine = 1;
data = readtable(datafile, opts);

% タイムスタンプを相対時間（秒）に変換
timestamps = datetime(data.Timestamp, 'InputFormat', 'yyyy/MM/dd HH:mm:ss.SSS');
start_time = timestamps(1);
time_seconds = seconds(timestamps - start_time);  % 開始点を0とした相対時間

% 10秒以降から18秒までのデータに制限（10秒以前は除外）
time_start = 10;  % 開始時間を10秒に設定
time_limit = 18;
valid_indices = (time_seconds >= time_start) & (time_seconds <= time_limit);
time_filtered = time_seconds(valid_indices);
force_x_raw = data.Force_X_N(valid_indices);
force_y_raw = data.Force_Y_N(valid_indices);
force_z_raw = data.Force_Z_N(valid_indices);

%% ローパスフィルタの設計と適用
% サンプリング周波数の計算
Fs = 1 / mean(diff(time_filtered));  % サンプリング周波数 [Hz]

% 1Hz以上を減衰するローパスフィルタの設計
cutoff_freq = 1.0;  % カットオフ周波数 [Hz]
filter_order = 4;   % フィルタ次数
[b, a] = butter(filter_order, cutoff_freq/(Fs/2), 'low');

% 各軸にフィルタを適用（前方・後方フィルタリングで位相遅れなし）
force_x_filtered = filtfilt(b, a, force_x_raw);
force_y_filtered = filtfilt(b, a, force_y_raw);
force_z_filtered = filtfilt(b, a, force_z_raw);

%% グラフ作成（2×2配置）
figure('Position', [100, 100, 900, 600]);

% サブプロット1: X軸外力のみ
subplot(2,2,1);
plot(time_filtered, force_x_filtered, 'r-', 'LineWidth', 2);
grid on;
xlabel('時間 [s]');
ylabel('X軸外力 [N]');
title('X軸外力成分（フィルタ適用後）');
xlim([time_start, time_limit]);
ylim([min(force_x_filtered)*1.1, max(force_x_filtered)*1.1]);

% サブプロット2: Y軸外力のみ
subplot(2,2,2);
plot(time_filtered, force_y_filtered, 'g-', 'LineWidth', 2);
grid on;
xlabel('時間 [s]');
ylabel('Y軸外力 [N]');
title('Y軸外力成分（フィルタ適用後）');
xlim([time_start, time_limit]);
ylim([min(force_y_filtered)*1.1, max(force_y_filtered)*1.1]);

% サブプロット3: Z軸外力のみ
subplot(2,2,3);
plot(time_filtered, force_z_filtered, 'b-', 'LineWidth', 2);
grid on;
xlabel('時間 [s]');
ylabel('Z軸外力 [N]');
title('Z軸外力成分（フィルタ適用後）');
xlim([time_start, time_limit]);
ylim([min(force_z_filtered)*1.1, max(force_z_filtered)*1.1]);

% サブプロット4: 全軸統合表示
subplot(2,2,4);
plot(time_filtered, force_x_filtered, 'r-', 'LineWidth', 2, 'DisplayName', 'Force X');
hold on;
plot(time_filtered, force_y_filtered, 'g-', 'LineWidth', 2, 'DisplayName', 'Force Y');
plot(time_filtered, force_z_filtered, 'b-', 'LineWidth', 2, 'DisplayName', 'Force Z');
grid on;
xlabel('時間 [s]');
ylabel('外力 [N]');
title('全軸外力統合表示（フィルタ適用後）');
legend('Location', 'best');
xlim([time_start, time_limit]);

% 全体タイトル
sgtitle('ペイロード外力推定結果（10-18秒、1Hzローパスフィルタ適用）', 'FontSize', 16, 'FontWeight', 'bold');

%% フーリエ変換解析
N = length(time_filtered);           % データ点数
f = (0:N/2-1) * Fs / N;             % 周波数軸 [Hz]

% 各軸のフーリエ変換（フィルタ適用後データ）
Y_x = fft(force_x_filtered);
Y_y = fft(force_y_filtered);
Y_z = fft(force_z_filtered);

% 振幅スペクトラムの計算（片側スペクトラム）
P_x = abs(Y_x(1:N/2)) * 2 / N;
P_y = abs(Y_y(1:N/2)) * 2 / N;
P_z = abs(Y_z(1:N/2)) * 2 / N;

% DC成分の補正
P_x(1) = P_x(1) / 2;
P_y(1) = P_y(1) / 2;
P_z(1) = P_z(1) / 2;

%% フーリエ変換結果の表示（第二の図）
figure('Position', [1000, 100, 900, 600]);

% X軸外力のFFT
subplot(3,1,1);
plot(f, P_x, 'r-', 'LineWidth', 2);
grid on;
xlabel('周波数 [Hz]');
ylabel('振幅');
title('X軸外力 フーリエ変換（フィルタ適用後）');
xlim([0, min(50, Fs/2)]);  % 0-50Hz または ナイキスト周波数まで

% Y軸外力のFFT
subplot(3,1,2);
plot(f, P_y, 'g-', 'LineWidth', 2);
grid on;
xlabel('周波数 [Hz]');
ylabel('振幅');
title('Y軸外力 フーリエ変換（フィルタ適用後）');
xlim([0, min(50, Fs/2)]);

% Z軸外力のFFT
subplot(3,1,3);
plot(f, P_z, 'b-', 'LineWidth', 2);
grid on;
xlabel('周波数 [Hz]');
ylabel('振幅');
title('Z軸外力 フーリエ変換（フィルタ適用後）');
xlim([0, min(50, Fs/2)]);

% 全体タイトル
sgtitle('外力成分 フーリエ変換スペクトラム（1Hzローパスフィルタ適用後）', 'FontSize', 16, 'FontWeight', 'bold');

%% フィルタ特性の表示（第三の図）
figure('Position', [100, 750, 600, 400]);

% フィルタの周波数応答
[H, w] = freqz(b, a, 1024, Fs);
semilogx(w, 20*log10(abs(H)), 'k-', 'LineWidth', 2);
grid on;
xlabel('周波数 [Hz]');
ylabel('ゲイン [dB]');
title('ローパスフィルタの周波数特性');
xlim([0.1, Fs/2]);
ylim([-60, 5]);
% カットオフ周波数にマーク
hold on;
semilogx(cutoff_freq, -3, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
text(cutoff_freq*1.5, -3, sprintf('%.1f Hz (-3dB)', cutoff_freq), 'FontSize', 10);

%% 統計情報表示（10-18秒間、フィルタ適用後）
fprintf('=== 外力データ統計情報（フィルタ適用後）===\n');
fprintf('表示時間範囲: %.1f - %.1f 秒\n', time_start, time_limit);
fprintf('データ点数: %d 点\n', sum(valid_indices));
fprintf('平均サンプリング周期: %.1f ms\n', mean(diff(time_filtered))*1000);
fprintf('サンプリング周波数: %.1f Hz\n', Fs);
fprintf('フィルタ: %d次Butterworthローパスフィルタ（カットオフ: %.1f Hz）\n', filter_order, cutoff_freq);
fprintf('\n');

% フィルタ適用前後の比較
fprintf('=== フィルタ適用前後の比較 ===\n');
fprintf('外力X軸:\n');
fprintf('  適用前 - 平均: %.3f N, 標準偏差: %.3f N\n', mean(force_x_raw), std(force_x_raw));
fprintf('  適用後 - 平均: %.3f N, 標準偏差: %.3f N\n', mean(force_x_filtered), std(force_x_filtered));
fprintf('外力Y軸:\n');
fprintf('  適用前 - 平均: %.3f N, 標準偏差: %.3f N\n', mean(force_y_raw), std(force_y_raw));
fprintf('  適用後 - 平均: %.3f N, 標準偏差: %.3f N\n', mean(force_y_filtered), std(force_y_filtered));
fprintf('外力Z軸:\n');
fprintf('  適用前 - 平均: %.3f N, 標準偏差: %.3f N\n', mean(force_z_raw), std(force_z_raw));
fprintf('  適用後 - 平均: %.3f N, 標準偏差: %.3f N\n', mean(force_z_filtered), std(force_z_filtered));
fprintf('\n');

fprintf('外力範囲（フィルタ適用後）:\n');
fprintf('  X軸: %.3f ~ %.3f N\n', min(force_x_filtered), max(force_x_filtered));
fprintf('  Y軸: %.3f ~ %.3f N\n', min(force_y_filtered), max(force_y_filtered));
fprintf('  Z軸: %.3f ~ %.3f N\n', min(force_z_filtered), max(force_z_filtered));

%% フーリエ変換統計情報
fprintf('\n=== フーリエ変換解析結果（フィルタ適用後）===\n');

% 各軸の主要周波数成分を検出
[~, idx_x] = max(P_x(2:end));  % DC成分を除く
[~, idx_y] = max(P_y(2:end));
[~, idx_z] = max(P_z(2:end));

dominant_freq_x = f(idx_x + 1);  % +1はDC成分を除いたため
dominant_freq_y = f(idx_y + 1);
dominant_freq_z = f(idx_z + 1);

fprintf('主要周波数成分:\n');
fprintf('  X軸: %.2f Hz (振幅: %.3f)\n', dominant_freq_x, P_x(idx_x + 1));
fprintf('  Y軸: %.2f Hz (振幅: %.3f)\n', dominant_freq_y, P_y(idx_y + 1));
fprintf('  Z軸: %.2f Hz (振幅: %.3f)\n', dominant_freq_z, P_z(idx_z + 1));
