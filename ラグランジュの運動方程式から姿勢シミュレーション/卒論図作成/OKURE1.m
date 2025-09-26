clear;
tic;

% パラメータ設定
filename = 'output44.csv'; % CSVファイル名を指定
tau = 0.1;       % 時定数[s] % ここで時定数を設定
dt = 0.01;      % データ間隔[s]
tspan = [0 1]; % シミュレーション時間

% CSVファイルを読み込む
T = readtable(filename);

% ... (CSVデータ処理部分 - 変更なし)
time_data = T{:, 1};
start_time = time_data(1);
rows_to_keep = time_data >= start_time;
T = T(rows_to_keep, :);
start_att = table2array(T(1, [2 3 4]));
motor_val_csv = table2array(T(:, [5 6 7 8]));
time_csv = table2array(T(:,1)) - start_time;
real_att = table2array(T(1:min(length(time_csv),tspan(2)*100+1), [2 3 4]));
real_time = table2array(T(1:min(length(time_csv),tspan(2)*100+1), 1))-start_time;

% 出力時間ベクトルの作成 (0.01秒間隔)
t_out = 0:dt:tspan(2);

% 結果を格納する配列を初期化
filtered_motor_val_csv = zeros(length(t_out), size(motor_val_csv, 2));

% 各列に対してフィルタ処理を適用
for j = 1:size(motor_val_csv, 2) % 列ごとに処理
    % 初期値を設定
    x_FO_prev = motor_val_csv(1, j); % 各列の最初の値を初期値とする
    for i = 2:length(t_out)
        dt_filter = t_out(i) - t_out(i-1);
        filtered_motor_val_csv(i, j) = LPF(dt_filter, interp1(time_csv,motor_val_csv(:, j),t_out(i)), x_FO_prev, tau);
        x_FO_prev = filtered_motor_val_csv(i, j); % 状態の更新
    end
end

% フィルタ前後の信号をプロット（確認用）
figure;
hold on;
for i = 1:size(motor_val_csv, 2)
    motor_val_csv_interp = interp1(time_csv, motor_val_csv(:, i), t_out, 'linear', 'extrap'); % 補間
    plot(t_out, motor_val_csv_interp, 'b-', 'DisplayName', ['Original Motor ', num2str(i)]);
    plot(t_out, filtered_motor_val_csv(:, i), 'g-', 'DisplayName', ['Filtered Motor ', num2str(i)]);
end
hold off;
xlabel('Time [s]');
ylabel('Motor Value');
legend;
title(['Motor Values with Lowpass Filter (Time Constant = ', num2str(tau), ' s)']); % タイトルを変更
grid on;

% 結果をテーブルに変換して保存 (オプション)
T_out = array2table([t_out', filtered_motor_val_csv]);
T_out.Properties.VariableNames = {'Time', 'Motor1_Filtered', 'Motor2_Filtered', 'Motor3_Filtered','Motor4_Filtered'}; % 必要に応じて列名を変更
writetable(T_out, 'filtered_motor_data.csv'); % 新しいCSVファイルに保存

toc;

function x_FO = LPF(T, diff_q_rot, x_FO_prev, tau)
    x_FO = zeros(size(diff_q_rot)); % 出力を初期化
    for i = 1:length(diff_q_rot) % MATLABのインデックスは1から始まる
        x_FO(i) = x_FO_prev(i) + (T/tau) * (diff_q_rot(i) - x_FO_prev(i));
    end
end