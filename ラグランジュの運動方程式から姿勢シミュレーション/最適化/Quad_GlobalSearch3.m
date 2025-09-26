clear;

tic1 = tic;

% パラメータの定義
tspan = [0 0.3]; % シミュレーション時間

% CSVデータを事前に読み込む
T = readtable('output45.csv');
start_time = 16.7; % 開始時間

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
L_x = 0.18; L_y = 0.285; % プロペラ間の距離

const1 = [m; g]; const2 = [L_x, L_y];


% 　調節するパラメータの初期値を入れる。
J_xx = 0.005; J_yy = 0.005; J_zz = 0.005; 
mu_1 = 0.001; mu_2 = 0.001;
OF_x = 0.0001; OF_y = 0.0001;  % 4つのプロペラの中心から、x y 方向の重心のオフセット

% 初期パラメータ値
params0 = [J_xx; J_yy; J_zz; mu_1; mu_2; OF_x; OF_y];

% 変更のない定数を構造体に格納
objective_args.tspan = tspan;
objective_args.Current0 = Current0;
objective_args.const1 = const1;
objective_args.const2 = const2;
objective_args.fdX = fdX;
objective_args.fR_BtoI = fR_BtoI;
objective_args.fP_BtoI = fP_BtoI;
objective_args.time_csv = time_csv;
objective_args.motor_val_csv = motor_val_csv;
objective_args.real_time = real_time;
objective_args.real_att = real_att;

% クロージャーを使って目的関数を作成
objective_fun = @(params) objective_function(params, objective_args);

% 最適化問題の作成
problem = createOptimProblem('fmincon', ... % fminconを使用する最適化問題
    'objective', objective_fun, ... % 目的関数
    'x0', params0, ...
    'lb', [0, 0, 0, 0, 0, -0.1, -0.1], ... % 各パラメータの下限
    'ub', [0.1, 0.1, 0.1, 1, 1, 0.1, 0.1]);        % 各パラメータの上限); % 初期値



% GlobalSearchオプションの設定
optionsGS = optimoptions('fmincon', ... % fmincon用のオプション
    'Display', 'iter', ... % 結果を逐次表示
    'MaxIterations', 300, ... % 最大反復回数
    'MaxFunctionEvaluations', 2000, ... % 最大関数評価回数
    'OptimalityTolerance', 1e-4); % 最適性許容誤差

gs = GlobalSearch( ...
    'NumTrialPoints', 500, ... % 試行点の総数を200以上に設定（デフォルト200以上推奨）
    'NumStageOnePoints', 200, ... % ステージ1の点数（デフォルトと一致）
    'StartPointsToRun', 'bounds-ineqs', ... % 初期点選択方法
    'MaxTime', 60*15, ... % 最大実行時間（秒）
    'Display', 'iter'); % 表示

% 最適化実行
[x_opt, fval, exitflag, output] = run(gs, problem);

% % GlobalSearchを実行
% params_estimated = run(gs, problem);

disp('最適化前のパラメータ');
disp(params0);
disp('最適化した後のパラメータ ');
disp(x_opt);

% 最適化後のパラメータでシミュレーションを実行
Simu_Val_optimized = [x_opt(1:3); const1; x_opt(4:5)]; % const1の正しい更新方法
offset_value_optimized = x_opt(6:7);

options = odeset('RelTol',1e-6,'AbsTol',1e-6);
[ta, Current] = ode45(@(t, Current) system_dynamics(t, Current, Simu_Val_optimized, offset_value_optimized, const2, fdX, fR_BtoI, fP_BtoI, time_csv, motor_val_csv), tspan, Current0, options);

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

toc(tic1);

% system_dynamics
function dCurrent = system_dynamics(t, Current, Simu_Val,offset_value, const2, fdX, fR_BtoI, fP_BtoI, time_csv, motor_val_csv)
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
    L_x = const2(1); L_y = const2(2); % プロペラ間の距離

    OF_x = offset_value(1);  OF_y = offset_value(2); 
    Lx_Rear = L_x/2 + OF_x;
    Lx_Front = L_x/2 - OF_x;
    Ly_Right = L_y/2 + OF_y;
    Ly_Left = L_y/2 - OF_y;

    L1 = (Lx_Rear^2 + Ly_Right^2)^(1/2);
    L2 = (Lx_Front^2 + Ly_Right^2)^(1/2);
    L3 = (Lx_Rear^2 + Ly_Left^2)^(1/2);
    L4 = (Lx_Front^2 + Ly_Left^2)^(1/2);

    motor_torq_roll = sum((motor_val .* [Ly_Right Ly_Right Ly_Left Ly_Left])' .* [-1 -1 1 1]' *0.022191707538092);
    motor_torq_pitch = sum((motor_val .* [Lx_Rear Lx_Front Lx_Rear Lx_Front])' .* [1 -1 1 -1]' *0.022191707538092);
    motor_torq_yaw = sum((motor_val .* [L1 L2 L3 L4])' .* [-1 1 1 -1]' * 0.002069627691412);
    motor_torq = [motor_torq_roll; motor_torq_pitch; motor_torq_yaw];

    F_b = [0; 0; 1] * 80 * 0.022191707538092 * 4;
    Tau_b = motor_torq;

    F_I = R_BtoI * F_b;
    Tau_I = P_BtoI * Tau_b;
    Q_I = [F_I; Tau_I];

    vars = {[X_I; Q_I; Simu_Val]};
    fdx_val = fdX(vars{:});
    dCurrent = fdx_val;
end

% objective_function (変更：二乗和を返すように変更)
function error = objective_function(params, objective_args)
    try
        % 構造体から引数を展開
        tspan = objective_args.tspan;
        Current0 = objective_args.Current0;
        const1 = objective_args.const1;
        const2 = objective_args.const2;
        fdX = objective_args.fdX;
        fR_BtoI = objective_args.fR_BtoI;
        fP_BtoI = objective_args.fP_BtoI;
        time_csv = objective_args.time_csv;
        motor_val_csv = objective_args.motor_val_csv;
        real_time = objective_args.real_time;
        real_att = objective_args.real_att;
    
        % const1の更新 (paramsを反映)
        Simu_Val = [params(1:3); const1; params(4:5)]; % const1の正しい更新方法
        offset_value = params(6:7);

        % Events オプションの設定
        options = odeset('RelTol', 1e-3, 'AbsTol', 1e-3, ...
                         'Events', @(t, Current) stop_condition(t, Current));

        % ODE の計算
        [ta, Current, te, ye, ie] = ode45(@(t, Current) ...
            system_dynamics(t, Current, Simu_Val, offset_value, const2, fdX, fR_BtoI, fP_BtoI, time_csv, motor_val_csv), ...
            tspan, Current0, options);

        % 計算が途中で停止した場合、error を大きな値に設定
        if ~isempty(te)
            warning('Constraint violated during ODE calculation.');
            error = 1e10;
            return;
        end

        sim_att = interp1(ta, Current(:, 4:6), real_time);
        nan_indices = isnan(sim_att);
        sim_att(nan_indices) = 0;
        
        error = sum(sum((sim_att - real_att).^2));
        disp(error);

    catch ME
        % warning(['Error in objective_function: ', ME.message]);
        % エラーが発生した場合、大きな値を返す
        error = 1e10;

    end
end


function [value, isterminal, direction] = stop_condition(t, Current)
    % 条件設定:閾値
    threshold = 4; % 閾値を設定（例: 10）
    condition = any(abs(Current(4:6)) > threshold);

    % イベントの出力
    value = ~condition; % 条件が満たされると 0 を返す
    isterminal = 1;     % 条件が満たされたら ODE を停止
    direction = 0;      % 条件がどの方向から満たされても停止
end
