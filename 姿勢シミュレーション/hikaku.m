clear;
tic;

% パラメータの定義
tspan = [0 1.2]; % シミュレーション時間

% CSVデータを事前に読み込む
T = readtable('output44.csv');
start_time = 21; % 開始時間

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

disp(['時刻 ', num2str(target_time), ' の時のフィルタ後の微分値 (移動平均): ', num2str(filtered_diff_value_at_target_time)]);

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

% 初期条件
Eta0 = start_att';
dEta0 = filtered_diff_value_at_target_time';
x0 = [Eta0; dEta0];

options = odeset('RelTol', 1e-1, 'AbsTol', 1e-2);
% 微分方程式の解を求める
% [t1, xx] = ode45(@(t, x) func1(t, x, time_csv, motor_val_csv), tspan, x0);
[t1, xx] = ode45(@(t, x) func1(t, x, time_csv, motor_val_csv), tspan, x0, options);

% ... (前略) 最適化計算、シミュレーション計算まで

% プロット (最適化後)

figure(1); clf; % figure(1)を作成またはクリア

% ロール角 (φ)
subplot(3, 1, 1); % 3行1列の1番目のsubplot
plot(real_time, real_att(:,1), 'b-', 'LineWidth', 1.5); % 実験値 (青実線)
hold on;
plot(t1, xx(:,1), 'r--', 'LineWidth', 1.5); % シミュレーション値 (赤破線)
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
plot(t1, xx(:,2), 'r--', 'LineWidth', 1.5); % シミュレーション値 (赤破線)
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
plot(t1, xx(:,3), 'r--', 'LineWidth', 1.5); % シミュレーション値 (赤破線)
hold off;
% title('ヨー角 (ψ)');
xlabel('Time [s]','FontSize',13);
ylabel(' Yaw Angle [rad]','FontSize',13);
legend('Experimental Data', 'Simulation Data','FontSize',10); % 凡例も英語表記に

grid on;

% ... (後略) objective_function, func1


function dx = func1(t, x, time_csv, motor_val_csv)
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
    motor_val
% x = [Phi, Theta, Psi, dPhi, dTheta, dPsi]
    Eta = x(1:3);    % Eta = [Phi, Theta, Psi]
    dEta = x(4:6);   % dEta = [dPhi, dTheta, dPsi]
    Phi = Eta(1);    Theta = Eta(2);    Psi = Eta(3);
    dPhi = dEta(1);  dTheta = dEta(2);  dPsi = dEta(3);
    g = 9.81; % 重力加速度 (m/s^2)
    m = 1.7; % 機体重量 (kg)
    % Ib = diag([0.001, 0.001, 0.001]); % 慣性モーメント (kg*m^2)
        Ib = diag([0.01, 0.01, 0.01]); % 慣性モーメント (kg*m^2)
        % Ib = diag([0.1, 0.1, 0.1]); % 慣性モーメント (kg*m^2)
    P = [Ib(1,1),0,-1/2*Ib(1,1)*sin(Theta);
        0,Ib(2,2)*cos(Phi)+1/2*Ib(2,2)*sin(Phi)*cos(Theta),0;
        0,0,-Ib(1,1)*sin(Theta)+1/2*Ib(2,2)*sin(Phi)*cos(Theta)-1/2*Ib(3,3)*cos(Phi)*cos(Theta)]; % 慣性モーメントの逆行列
    r = 0.1685*(ones(1, 4)); % モータ軸の距離 (m)
    Jr = 1/2 * m * r.^2; % ジャイロ効果の慣性モーメント (スカラー)
    Omegar = [100; 100; 100; 100]; % ロータの角速度
        h1 =  -1/2*Ib(1,1)*dTheta*dPsi*cos(Theta)+Ib(2,2)*sin(Phi)*(1/2*dTheta*cos(Phi)+1/2*dPsi*sin(Phi)*cos(Theta))-Ib(2,2)*cos(Phi)*(1/2*dPsi*cos(Phi)*cos(Theta)-1/2*dTheta*sin(Phi)) -Ib(3,3)*cos(Phi)*(1/2*dTheta*sin(Theta)-1/2*dPsi*cos(Phi)*cos(Theta))-Ib(3,3)*sin(Phi)*(1/2*dTheta*cos(Phi)+1/2*dPsi*sin(Phi)*cos(Theta));%+ Jr * Omegar * dTheta;

        h2 =  Ib(2,2)*(-dPhi*dTheta*sin(Phi)+1/2*dPhi*dTheta*cos(Phi)*cos(Theta)+sin(Theta)*(1/2*dTheta*cos(Phi)+1/2*dPsi*cos(Theta)*sin(Phi))-cos(Theta)*sin(Phi)*(1/2*dPsi*cos(Theta)*sin(Phi)+1/2*dTheta*cos(Phi)))+Ib(3,3)*(1/2*dPhi*dTheta*cos(Phi)+sin(Theta)*cos(Phi)*(1/2*dTheta*sin(Phi)-1/2*dPsi*cos(Phi)*cos(Theta)));% - Jr * Omegar * dPhi  ;

        h3 =   Ib(1,1)*(-dTheta*dPsi*cos(Theta)+dPsi*sin(Theta))-Ib(2,2)*cos(Theta)*sin(Phi)*(1/2*dTheta*cos(Phi)+1/2*dPsi*cos(Theta)*sin(Phi))+Ib(3,3)*(1/2*dTheta*dPsi*sin(Theta)*cos(Phi)+cos(Theta)*cos(Phi)*(1/2*dTheta*sin(Phi)-1/2*dPsi*cos(Phi)*cos(Theta)));


        h = [ h1;h2;h3];
   
    % 
    % etad=[0;0;pi*0/180];  %目標角
    % kp = [5; 16; 50]; % 角度制御の比例ゲイン
    % kd = [6; 6; 5]; % 角度制御の微分ゲイン
    % 
    % 
    % 
    % input_motor = -kp .* (Eta-etad) - kd .* dEta; % 551~1023の間での実機の入力にプラスマイナスする値
    % % disp(input_motor);
    % input_sim = input_motor*100/(1023-551);  % 0~100の間でのシミュレーションの入力
    % motor_torq = 4*diag([0.1425,0.090,-1])*(diag([0.022191707538092,0.022191707538092,3.533461241187303e-04]) * input_sim);  % 重心からの距離(m) * モーターの生み出す力(N)

    motor_torq_roll =  sum(motor_val *diag([-1 -1 1 1])*0.022191707538092*0.1425);
    motor_torq_pitch =  sum(motor_val *diag([1 -1 1 -1])*0.022191707538092*0.090);
    motor_torq_yaw =  sum(motor_val *diag([-1 1 1 -1])*3.533461241187303e-04);
    motor_torq = [motor_torq_roll;motor_torq_pitch;motor_torq_yaw];


    Tau = motor_torq;
 
    % d2x (加速度成分)
    d2x = P\ (Tau - h);
    % dx = [dEta; d2x] -> [dPhi, dTheta, dPsi, d2Phi, d2Theta, d2Psi]
    dx = [dEta; d2x];
end