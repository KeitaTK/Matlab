%  一次遅れを追加
clear;

tic;

% 
% % CSVデータを事前に読み込む
T = readtable('output45.csv');
start_time = 16 ; % 開始時間

% T = readtable('output44.csv');
% start_time = 21; % 開始時間


tspan = [0 1]; % シミュレーション時間
numStartPoints = 500; % 試行点の総数


% 初期値を求める%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time_data = T{:, 1};
euler_data = T{:, 2:4};

% 初期姿勢角にローパスフィルタをかける。
euler_lowpass = zeros(size(euler_data));
for i = 1:3
    euler_lowpass(:,i) = lowpass(euler_data(:,i),20,100);
end 


% 微分値(deuler)を計算
diff_value = zeros(size(euler_data));
for j = 1:size(euler_data, 2)
    for i = 2:length(time_data)-1
        diff_value(i, j) = (euler_data(i+1, j) - euler_data(i-1, j)) / (time_data(i+1) - time_data(i-1));
    end
    diff_value(1,j) = (euler_data(2, j) - euler_data(1, j)) / (time_data(2) - time_data(1));
    diff_value(end,j) = (euler_data(end, j) - euler_data(end-1, j)) / (time_data(end) - time_data(end-1));
end

% % 微分値(deuler)にローパスフィルタをかける。
deuler_lowpass = zeros(size(diff_value));
for i = 1:3
    deuler_lowpass(:,i) = lowpass(diff_value(:,i),20,100);
end                                   

% 時刻がスタートの時のeulerとdeulerを求める。
target_time = start_time;
[~, target_index] = min(abs(time_data - target_time));
filtered_diff_value_at_target_time = deuler_lowpass(target_index, :);
filtered_euler_at_target_time = euler_lowpass(target_index, :);

disp(['時刻 ', num2str(target_time), ' の時の微分値: ', num2str(filtered_diff_value_at_target_time)]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% テーブルの最初の列を取得 (時間データ)
time_data = T{:, 1};

% 論理インデックスを作成
rows_to_keep = time_data >= start_time;

% 条件を満たす行のみを保持
T = T(rows_to_keep, :);
start_att = table2array(T(1, [2 3 4]));
motor_val_csv = table2array(T(:, [5 6 7 8])); % CSVのモーター入力データ
time_csv = time_data(rows_to_keep)-start_time;
real_att = table2array(T(1:tspan(2)*100, [2 3 4]));
real_time = table2array(T(1:tspan(2)*100, 1))-start_time;

%% 
% 運動方程式と回転行列、角速度ベクトルをロード

load("dX.mat", "fdX");
load("R_BtoI.mat", "fR_BtoI");
load("Omega.mat", "fOmega");
load("P_BtoI.mat", "fP_BtoI");
load("P_ItoB.mat", "fP_ItoB");
%% 
% 初期値を設定

X0 = [0; 0; 0];    % 位置
dX0 = [0; 0; 0];   % 速度

eta0 = filtered_euler_at_target_time';   % 角度
deta0 = filtered_diff_value_at_target_time';   % 角速度

motor_val_filtered0 = [0;0;0;0]; % フィルタ出力の初期値
Current0 = [X0; eta0; dX0; deta0];


figure(1); clf;
subplot(2,1,1); % 2行1列の1番目

hold on;
plot(time_data, euler_data(:,3));
plot(time_data, euler_lowpass(:,3));
hold off;
plot(time, euler(:,2));
plot(time, euler(:,3));
title('Euler Angles');
xlabel('時間 (秒)');
ylabel('姿勢角(度)');


subplot(2,1,2); % 2行1列の2番目
plot(time, motor(:,1));
hold on;
plot(time, motor(:,2));
plot(time, motor(:,3));
plot(time, motor(:,4));
title('Motor Outputs');

figure(2); clf;
subplot(2,1,1); % 2行1列の1番目
plot(time, euler(:,2));
hold on;
plot(time, euler_lowpass(:,2));