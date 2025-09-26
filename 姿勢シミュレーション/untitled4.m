% メインスクリプト

clear;
tic;

% パラメータの定義
tspan = [0 1.2]; % シミュレーション時間
% CSVデータを事前に読み込む
T = readtable('output44.csv');
start_time = 20; % 開始時間

% 初期値を求める
time_data = T{:, 1};
value_data = T{:, 2:4};

% 微分値を計算
diff_value = zeros(size(value_data));
for j = 1:size(value_data, 2)
    for i = 2:length(time_data)-1
        diff_value(i, j) = (value_data(i+1, j) - value_data(i-1, j)) / (time_data(i+1) - time_data(i-1));
    end
    diff_value(1,j) = (value_data(2, j) - value_data(1, j)) / (time_data(2) - time_data(1));
    diff_value(end,j) = (value_data(end, j) - value_data(end-1, j)) / (time_data(end) - time_data(end-1));
end

% 移動平均フィルタ
window_size = 5;
filtered_diff_value = zeros(size(diff_value));
for j = 1:size(diff_value, 2)
    filtered_diff_value(:,j) = movmean(diff_value(:,j), window_size);
end

% 時刻がスタートの時のフィルタ後の微分値を求める
target_time = start_time;
[~, target_index] = min(abs(time_data - target_time));
filtered_diff_value_at_target_time = filtered_diff_value(target_index, :);
disp(['時刻 ', num2str(target_time), ' の時のフィルタ後の微分値 (移動平均): ', num2str(filtered_diff_value_at_target_time)]);

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

% 初期条件
Eta0 = start_att';
dEta0 = filtered_diff_value_at_target_time';
x0 = [Eta0; dEta0];

% 初期値
Ib_initial = [0.003, 0.003, 0.005];

% 最適化を実行 (lsqnonlinを使用)
options = optimoptions('lsqnonlin','Display','iter');
[Ib_optimized, resnorm,residual] = lsqnonlin(@(Ib_values) objective_function_vec(Ib_values, time_csv, motor_val_csv, real_time, real_att,start_att, filtered_diff_value_at_target_time), Ib_initial,[],[],options);

disp(['最適化された慣性モーメント: ', num2str(Ib_optimized)]);
disp(['残差二乗和: ', num2str(resnorm)]);

% 最適化後のシミュレーション
Ib = diag(Ib_optimized);
[t1, xx] = ode45(@(t, x) func1(t, x, time_csv, motor_val_csv, Ib), tspan, x0);


% プロット (最適化後)
figure(1); clf; hold on;
subplot(2,1,1);
plot(real_time, real_att(:,1));
title('ロール角 (φ) 実験値');
xlabel('時間 (秒)');
ylabel('角度 (rad)');
ylim_upper1 = ylim;
xlim_upper1 = xlim;
subplot(2,1,2);
plot(t1, xx(:,1));
title('ロール角 (φ) シミュレーション値(最適化後)');
xlabel('時間 (秒)');
ylabel('角度 (rad)');
ylim(ylim_upper1);
xlim(xlim_upper1);

%以下同様にfigure(2),(3)も作成

% ... (他のプロットも同様に)

toc;

% 目的関数 (ベクトル版)
function error_vec = objective_function_vec(Ib_values, time_csv, motor_val_csv, real_time, real_att,start_att, filtered_diff_value_at_target_time)
        % Ib_values: 最適化する慣性モーメントの値 [Ib(1,1), Ib(2,2), Ib(3,3)]
    % time_csv, motor_val_csv, real_time, real_att: 実験データ
    % start_att, filtered_diff_value_at_target_time: 初期値
        % 慣性モーメントを更新
    Ib = diag(Ib_values);
    % 初期条件
    Eta0 = start_att';
    dEta0 = filtered_diff_value_at_target_time';
    x0 = [Eta0; dEta0];
        tspan = [0 1.2];
    options = odeset('RelTol', 1e-1, 'AbsTol', 1e-2);

    % シミュレーションを実行
    [t1, xx] = ode45(@(t, x) func1(t, x, time_csv, motor_val_csv, Ib), tspan, x0, options);

    % シミュレーション結果と実験値の差を計算 (各時刻のロール、ピッチ、ヨーの誤差)
        sim_att = interp1(t1, xx(:, 1:3), real_time); % シミュレーション結果を実験データの時間軸に補間
    if size(sim_att,1) ~= size(real_att,1)
        error_vec = 1e10*ones(1,size(real_att,1)*size(real_att,2));
        return
    end
    error_vec = (sim_att - real_att);
    error_vec = reshape(error_vec,[1,size(real_att,1)*size(real_att,2)]);
end

% 微分方程式
function dx = func1(t, x, time_csv, motor_val_csv, Ib)
    % 時間 t に対応するモーター入力値を線形補間 (変更なし)
    if isempty(time_csv)
        motor_val = [0,0,0,0]; % CSVデータがない場合の処理
    else
        for i = 1:length(time_csv)-1
            if t >= time_csv(i) && t <= time_csv(i+1)
                motor_val = motor_val_csv(i,:) + (motor_val_csv(i+1,:) - motor_val_csv(i,:)) * (t - time_csv(i)) / (time_csv(i+1) - time_csv(i));
                break;
            elseif t < time_csv(1)
                motor_val = motor_val_csv(1,:);
                break;
            elseif t > time_csv(end)
                motor_val = motor_val_csv(end,:);
                break;
            end
        end
    end

    Eta = x(1:3);
    dEta = x(4:6);
    Phi = Eta(1);    Theta = Eta(2);    Psi = Eta(3);
    dPhi = dEta(1);  dTheta = dEta(2);  dPsi = dEta(3);
    g = 9.81;
    m = 1.7;
        %Ib = diag([0.003, 0.003, 0.005]); % 慣性モーメント (kg*m^2) %最適化パラメータ
    P = [Ib(1,1),0,-1/2*Ib(1,1)*sin(Theta);
        0,Ib(2,2)*cos(Phi