clear;

tic;

% パラメータの定義
tspan = [0 1.5]; % シミュレーション時間

% CSVデータを事前に読み込む
T = readtable('output45.csv');
start_time = 16.35; % 開始時間

% 初期値を求める%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
window_size = 5; % 移動平均の窓幅
filtered_diff_value = zeros(size(diff_value));
for j = 1:size(diff_value, 2)
    filtered_diff_value(:,j) = movmean(diff_value(:,j), window_size);
end

% 時刻がスタートの時のフィルタ後の微分値を求める (前述のコードと同様)
target_time = start_time;
[~, target_index] = min(abs(time_data - target_time));
filtered_diff_value_at_target_time = filtered_diff_value(target_index, :);

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

eta0 = start_att';   % 角度
deta0 = filtered_diff_value_at_target_time';   % 角速度

Current0 = [X0; eta0; dX0; deta0];
%% 
% シミュレーションの環境設定
tspan = 0:0.01:tspan(2);      % シミュレーション時間
%% 

% 定数に値を入れる 
m = 1.36; g = 9.81;
% 　調節するパラメータの初期値を入れる。
J_xx = 0.005; J_yy = 0.005; J_zz = 0.005; 
mu_1 = 0.001; mu_2 = 0.001;
const = [m; g];
pre_palam = [J_xx; J_yy; J_zz; mu_1; mu_2];

% 4つのプロペラの中心から、x y 方向の重心のオフセット
L_y = 0.285;
OF_y = 0.0001;
L_x = 0.18;
OF_x = 0.0001;

Lx_Rear = L_x/2 + OF_x;
Lx_Front = L_x/2 - OF_x;
Ly_Right = L_y/2 + OF_y;
Ly_Left = L_y/2 - OF_y;

L1 = (Lx_Rear^2 + Ly_Right^2)^(1/2);
L2 = (Lx_Front^2 + Ly_Right^2)^(1/2);
L3 = (Lx_Rear^2 + Ly_Left^2)^(1/2);
L4 = (Lx_Front^2 + Ly_Left^2)^(1/2);

prop_position = [Lx_Rear; Lx_Front; Ly_Right; Ly_Left; L1; L2; L3; L4];

% 初期パラメータ値
params0 = pre_palam; % 現在のconstの値を初期値とする

% 最適化
options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt', ...
    'StepTolerance', 1e-9, 'FunctionTolerance', 1e-8); % 値を大きくする
params_estimated = lsqnonlin(@(params) objective_function(params, tspan, Current0, const, prop_position, fdX, fR_BtoI, fP_BtoI, time_csv, motor_val_csv, real_time, real_att), params0,[],[],options);
disp('最適化前のパラメータ');
disp(params0);
disp('最適化した後のパラメータ ');
disp(params_estimated);

% 最適化後のパラメータでシミュレーションを実行
const_optimized = [params_estimated(1:3);const;params_estimated(4:5)];
options = odeset('RelTol',1e-6,'AbsTol',1e-6);
[ta, Current] = ode45(@(t, Current) system_dynamics(t, Current, const_optimized, prop_position, fdX, fR_BtoI, fP_BtoI, time_csv, motor_val_csv), tspan, Current0,options);

% 結果のプロット (最適化前後の比較)
% ... (プロットコードは元のコードを参考に修正)
% %% 
% % ode45でシミュレーション
% 
% options = odeset('RelTol',1e-6,'AbsTol',1e-6);
% [ta, Current] = ode45(@(t, Current) system_dynamics(t, Current, const, setting_val, fdX, fR_BtoI, fOmega, fP_BtoI, fP_ItoB, time_csv, motor_val_csv), tspan, Current0,options);
%% 
% シミュレーション結果の取り出し

figure(1); clf; % figure(1)を作成またはクリア

% ロール角 (φ)
subplot(3, 1, 1); % 3行1列の1番目のsubplot
plot(real_time, real_att(:,1), 'b-', 'LineWidth', 1.5); % 実験値 (青実線)
hold on;
plot(ta, Current(:,4), 'r--', 'LineWidth', 1.5); % シミュレーション値 (赤破線)
hold off;
% title('(a) I_xx=I_yy=I_zz=0.001','FontSize',14);
xlabel('Time [s]','FontSize',13);
ylabel('Roll Angle [rad]','FontSize',13);
legend('Experimental Data', 'Simulation Data','FontSize',10); % 凡例も英語表記に

grid on;

% ピッチ角 (θ)
subplot(3, 1, 2); % 3行1列の2番目のsubplot
plot(real_time, real_att(:,2), 'b-', 'LineWidth', 1.5); % 実験値 (青実線)
hold on;
plot(ta, Current(:,5), 'r--', 'LineWidth', 1.5); % シミュレーション値 (赤破線)
hold off;
% title('ピッチ角 (θ)');
xlabel('Time [s]','FontSize',13);
ylabel('Pitch Angle [rad]','FontSize',13);
legend('Experimental Data', 'Simulation Data','FontSize',10); % 凡例も英語表記に

grid on;

% ヨー角 (ψ)
subplot(3, 1, 3); % 3行1列の3番目のsubplot
plot(real_time, real_att(:,3), 'b-', 'LineWidth', 1.5); % 実験値 (青実線)
hold on;
plot(ta, Current(:,6), 'r--', 'LineWidth', 1.5); % シミュレーション値 (赤破線)
hold off;
% title('ヨー角 (ψ)');
xlabel('Time [s]','FontSize',13);
ylabel(' Yaw Angle [rad]','FontSize',13);
legend('Experimental Data', 'Simulation Data','FontSize',10); % 凡例も英語表記に

grid on;



figure(2); clf; hold on;

subplot(3,1,1);
plot(ta, Current(:,10));
title('dPhi (dPhi)');
xlabel('時間 (秒)');
ylabel('角速度 (rad/s)');

subplot(3,1,2);
plot(ta, Current(:,11));
title('dTheta (dTheta)');
xlabel('時間 (秒)');
ylabel('角速度 (rad/s)');

subplot(3,1,3);
plot(ta, Current(:,12));
title('dPsi (dPsi)');
xlabel('時間 (秒)');
ylabel('角速度 (rad/s)');

figure(3); clf; hold on;
subplot(3,1,1);
plot(ta, Current(:,1));
title('位置(x)');
xlabel('時間 (秒)');
ylabel(' 位置(m)');
 
subplot(3,1,2);
plot(ta, Current(:,2));
title('位置(y)');
xlabel('時間 (秒)');
ylabel(' 位置(m)');
 
subplot(3,1,3);
plot(ta, Current(:,3));
title('位置(z)');
xlabel('時間 (秒)');
ylabel(' 位置(m)');

figure(4); clf; hold on;

subplot(3,1,1);
plot(ta, Current(:,7));
title('速度 (dx)');
xlabel('時間 (秒)');
ylabel('速度 (m/s)');

subplot(3,1,2);
plot(ta, Current(:,8));
title('速度 (dy)');
xlabel('時間 (秒)');
ylabel('速度 (m/s)');

subplot(3,1,3);
plot(ta, Current(:,9));
title('速度 (dz)');
xlabel('時間 (秒)');
ylabel('速度 (m/s)');

toc;

% system_dynamics
function dCurrent = system_dynamics(t, Current, const, prop_position, fdX, fR_BtoI, fP_BtoI, time_csv, motor_val_csv)
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

    Eta_I = Current(4:6);
    X_I = Current;

    % 各種変換行列を作る
    R_BtoI = fR_BtoI(Eta_I);
    P_BtoI = fP_BtoI(Eta_I);

    % 4つのプロペラの中心から、x y 方向の重心のオフセット
    L_y = 0.285;
    OF_y = 0.0001;
    L_x = 0.18;
    OF_x = 0.0001;

    Lx_Rear = prop_position(1);
    Lx_Front = prop_position(2);
    Ly_Right = prop_position(3);
    Ly_Left = prop_position(4);

    L1 = prop_position(5);
    L2 = prop_position(6);
    L3 = prop_position(7);
    L4 = prop_position(8);


    motor_torq_roll = sum((motor_val .* [Ly_Right Ly_Right Ly_Left Ly_Left])' .* [-1 -1 1 1]' *0.022191707538092);
    motor_torq_pitch = sum((motor_val .* [Lx_Rear Lx_Front Lx_Rear Lx_Front])' .* [1 -1 1 -1]' *0.022191707538092);
    motor_torq_yaw = sum((motor_val .* [L1 L2 L3 L4])' .* [-1 1 1 -1]' * 0.002069627691412);
    motor_torq = [motor_torq_roll; motor_torq_pitch; motor_torq_yaw];

    F_b = [0; 0; 1] * 80 * 0.022191707538092 * 4;
    Tau_b = motor_torq;

    F_I = R_BtoI * F_b;
    Tau_I = P_BtoI * Tau_b;
    Q_I = [F_I; Tau_I];

    vars = {[X_I; Q_I; const]};
    fdx_val = fdX(vars{:});
    dCurrent = fdx_val;
end

function error = objective_function(params, tspan, Current0, const, prop_position, fdX, fR_BtoI, fP_BtoI, time_csv, motor_val_csv, real_time, real_att)

    const = [params(1:3);const;params(4:5)];  %paramsとを変数constに代入
    options = odeset('RelTol',1e-6,'AbsTol',1e-8);
    [ta, Current] = ode45(@(t, Current) system_dynamics(t, Current, const, prop_position, fdX, fR_BtoI, fP_BtoI, time_csv, motor_val_csv), tspan, Current0,options);

    % シミュレーション結果と実験データを比較
    sim_att = interp1(ta, Current(:, 4:6), real_time); % シミュレーション結果を実験時間で補間
    
    % NaNの処理を追加
    nan_indices = isnan(sim_att);
    sim_att(nan_indices) = 0; % NaNを0で置き換え
    
    error = sum(sum((sim_att - real_att).^2)); % 姿勢データの差の二乗和
    % disp(error);
end
