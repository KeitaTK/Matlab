%% External Force Data Visualization Program (18 seconds, 4-panel display)
clear; clc; close all;

% CSV file loading
% filename = '20250721_192836_EF.csv';  % Without payload
% filename = '20250721_194808_EF.csv';  % With payload
% filename = '20250804_160030_EF.csv';  % ノイズフィルタあり、吊り荷なし
filename = '20250804_161747_EF.csv';  % ノイズフィルタあり、吊り荷あり
opts = detectImportOptions(filename);
opts.VariableNamesLine = 1;
data = readtable(filename, opts);

% Convert timestamp to relative time (seconds)
timestamps = datetime(data.Timestamp, 'InputFormat', 'yyyy/MM/dd HH:mm:ss.SSS');
start_time = timestamps(1);
time_seconds = seconds(timestamps - start_time);  % Relative time from start

% Limit data to 18 seconds
time_limit =28;
valid_indices = time_seconds <= time_limit;
time_filtered = time_seconds(valid_indices);
force_x_filtered = data.Force_X_N(valid_indices);
force_y_filtered = data.Force_Y_N(valid_indices);
force_z_filtered = data.Force_Z_N(valid_indices);

%% Graph creation (2×2 layout) with large fonts
figure('Position', [100, 100, 1200, 900]);

% Set default font sizes for all plots
set(0, 'DefaultAxesFontSize', 14);          % Axis labels and tick labels
set(0, 'DefaultTextFontSize', 16);          % General text
set(0, 'DefaultLegendFontSize', 14);        % Legend font size

% Subplot 1: X-axis force only
subplot(2,2,1);
plot(time_filtered, force_x_filtered, 'r-', 'LineWidth', 2.5);
grid on; grid minor;
set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);
xlabel('Time [s]', 'FontSize', 16, 'FontWeight', 'bold');
ylabel('X-axis Force [N]', 'FontSize', 16, 'FontWeight', 'bold');
title('X-axis Force Component', 'FontSize', 18, 'FontWeight', 'bold');
xlim([0, time_limit]);
ylim([min(force_x_filtered)*1.15, max(force_x_filtered)*1.15]);
set(gca, 'FontSize', 14, 'LineWidth', 1.2);

% Subplot 2: Y-axis force only
subplot(2,2,2);
plot(time_filtered, force_y_filtered, 'g-', 'LineWidth', 2.5);
grid on; grid minor;
set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);
xlabel('Time [s]', 'FontSize', 16, 'FontWeight', 'bold');
ylabel('Y-axis Force [N]', 'FontSize', 16, 'FontWeight', 'bold');
title('Y-axis Force Component', 'FontSize', 18, 'FontWeight', 'bold');
xlim([0, time_limit]);
ylim([min(force_y_filtered)*1.15, max(force_y_filtered)*1.15]);
set(gca, 'FontSize', 14, 'LineWidth', 1.2);

% Subplot 3: Z-axis force only
subplot(2,2,3);
plot(time_filtered, force_z_filtered, 'b-', 'LineWidth', 2.5);
grid on; grid minor;
set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);
xlabel('Time [s]', 'FontSize', 16, 'FontWeight', 'bold');
ylabel('Z-axis Force [N]', 'FontSize', 16, 'FontWeight', 'bold');
title('Z-axis Force Component', 'FontSize', 18, 'FontWeight', 'bold');
xlim([0, time_limit]);
ylim([min(force_z_filtered)*1.15, max(force_z_filtered)*1.15]);
set(gca, 'FontSize', 14, 'LineWidth', 1.2);

% Subplot 4: All-axis integrated display
subplot(2,2,4);
plot(time_filtered, force_x_filtered, 'r-', 'LineWidth', 2.5, 'DisplayName', 'Force X');
hold on;
plot(time_filtered, force_y_filtered, 'g-', 'LineWidth', 2.5, 'DisplayName', 'Force Y');
plot(time_filtered, force_z_filtered, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Force Z');
grid on; grid minor;
set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);
xlabel('Time [s]', 'FontSize', 16, 'FontWeight', 'bold');
ylabel('External Force [N]', 'FontSize', 16, 'FontWeight', 'bold');
title('All-axis Force Integration', 'FontSize', 18, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 14, 'FontWeight', 'bold');
xlim([0, time_limit]);
set(gca, 'FontSize', 14, 'LineWidth', 1.2);

% Adjust subplot spacing for better readability
set(gcf, 'PaperPositionMode', 'auto');

% Save high-quality figure
print(gcf, 'External_Force_Estimation_Results.png', '-dpng', '-r300');
fprintf('\nFigure saved as: External_Force_Estimation_Results.png (300 DPI)\n');

%% Statistical Information Display (18 seconds) - English Version
fprintf('=== External Force Data Statistics ===\n');
fprintf('Display time range: 0 - %.1f seconds\n', time_limit);
fprintf('Number of data points: %d points\n', sum(valid_indices));
fprintf('Average sampling period: %.1f ms\n', mean(diff(time_filtered))*1000);
fprintf('\n');
fprintf('Force X-axis - Mean: %.3f N, Std Dev: %.3f N\n', mean(force_x_filtered), std(force_x_filtered));
fprintf('Force Y-axis - Mean: %.3f N, Std Dev: %.3f N\n', mean(force_y_filtered), std(force_y_filtered));
fprintf('Force Z-axis - Mean: %.3f N, Std Dev: %.3f N\n', mean(force_z_filtered), std(force_z_filtered));
fprintf('\n');
fprintf('Force Range:\n');
fprintf('  X-axis: %.3f ~ %.3f N\n', min(force_x_filtered), max(force_x_filtered));
fprintf('  Y-axis: %.3f ~ %.3f N\n', min(force_y_filtered), max(force_y_filtered));
fprintf('  Z-axis: %.3f ~ %.3f N\n', min(force_z_filtered), max(force_z_filtered));

%% Additional Performance Metrics (English)
fprintf('\n=== Performance Analysis ===\n');
fprintf('RMS Values:\n');
fprintf('  X-axis RMS: %.3f N\n', rms(force_x_filtered));
fprintf('  Y-axis RMS: %.3f N\n', rms(force_y_filtered));
fprintf('  Z-axis RMS: %.3f N\n', rms(force_z_filtered));
fprintf('\n');
fprintf('Peak-to-Peak Values:\n');
fprintf('  X-axis P-P: %.3f N\n', max(force_x_filtered) - min(force_x_filtered));
fprintf('  Y-axis P-P: %.3f N\n', max(force_y_filtered) - min(force_y_filtered));
fprintf('  Z-axis P-P: %.3f N\n', max(force_z_filtered) - min(force_z_filtered));
