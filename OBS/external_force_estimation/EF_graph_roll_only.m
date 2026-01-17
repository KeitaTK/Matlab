%% Roll Axis External Force Visualization Program
clear; clc; close all;

% CSV file loading
% filename = '20250721_192836_EF.csv';  % Without payload
% filename = '20250721_194808_EF.csv';  % With payload
% filename = '20250804_160030_EF.csv';  % ノイズフィルタあり、吊り荷なし
filename = '20250804_161747_EF.csv';  % ノイズフィルタあり、吊り荷あり
% Build data file path relative to this script
script_dir = fileparts(mfilename('fullpath'));
datafile = fullfile(script_dir, '..', 'data', filename);
opts = detectImportOptions(datafile);
opts.VariableNamesLine = 1;
data = readtable(datafile, opts);

% Convert timestamp to relative time (seconds)
timestamps = datetime(data.Timestamp, 'InputFormat', 'yyyy/MM/dd HH:mm:ss.SSS');
start_time = timestamps(1);
time_seconds = seconds(timestamps - start_time);

% Time range settings (data start is 0s)
time_start = 12;    % Start time [s]
time_end = 28;     % End time [s]

% Limit data to specified time range
valid_indices = (time_seconds >= time_start) & (time_seconds <= time_end);
time_filtered = time_seconds(valid_indices);
force_roll_filtered = data.Force_X_N(valid_indices);

%% Graph creation with large fonts
figure('Position', [100, 100, 800, 600]);

% Set default font sizes
set(0, 'DefaultAxesFontSize', 14);
set(0, 'DefaultTextFontSize', 16);

% Plot Roll axis force
plot(time_filtered, force_roll_filtered, 'r-', 'LineWidth', 2.5);
grid on; grid minor;
set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);
xlabel('Time [s]', 'FontSize', 16, 'FontWeight', 'bold');
ylabel('Roll Force [N]', 'FontSize', 16, 'FontWeight', 'bold');
title('Roll Axis External Force', 'FontSize', 18, 'FontWeight', 'bold');
xlim([time_start, time_end]);
ylim([min(force_roll_filtered)*1.15, max(force_roll_filtered)*1.15]);
set(gca, 'FontSize', 14, 'LineWidth', 1.2);

% Save high-quality figure
print(gcf, 'Roll_Force_Estimation_Results.png', '-dpng', '-r300');
fprintf('\nFigure saved as: Roll_Force_Estimation_Results.png (300 DPI)\n');

%% Statistical Information Display - Roll Axis Only
fprintf('=== Roll Axis External Force Statistics ===\n');
fprintf('Display time range: %.1f - %.1f seconds\n', time_start, time_end);
fprintf('Number of data points: %d points\n', sum(valid_indices));
fprintf('Average sampling period: %.1f ms\n', mean(diff(time_filtered))*1000);
fprintf('\n');
fprintf('Roll Force - Mean: %.3f N, Std Dev: %.3f N\n', mean(force_roll_filtered), std(force_roll_filtered));
fprintf('\n');
fprintf('Force Range:\n');
fprintf('  Roll: %.3f ~ %.3f N\n', min(force_roll_filtered), max(force_roll_filtered));

%% Additional Performance Metrics
fprintf('\n=== Performance Analysis ===\n');
fprintf('RMS Value: %.3f N\n', rms(force_roll_filtered));
fprintf('Peak-to-Peak Value: %.3f N\n', max(force_roll_filtered) - min(force_roll_filtered));
