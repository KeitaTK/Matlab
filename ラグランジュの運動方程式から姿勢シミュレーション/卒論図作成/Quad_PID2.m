%  一次遅れを追加
clear;

tic;

% 
% % CSVデータを事前に読み込む
T = readtable('output45.csv');
start_time = 16 ; % 開始時間


tspan = 0:0.01:35; % シミュレーション時間

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

eta0 = [0; 0; 0];   % 角度
deta0 = [2.19; 2.78; 1.81];   % 角速度

motor_val_filtered0 = [0;0;0;0]; % フィルタ出力の初期値
Current0 = [X0; eta0; dX0; deta0];
%% 
% 定数に値を入れる 
m = 1.36; g = 9.81;
L_x = 0.18; L_y = 0.285; % プロペラ間の距離

const1 = [m; g]; const2 = [L_x, L_y];


params = [0.002213506;
            0.002635199;
            0.018674491;
            0.540295497;
            0.002622546;
            -0.000201036;
            0.000795337;
            0.225357444;

            ];


disp('パラメータ:J_xx; J_yy; J_zz; mu_1; mu_2; OF_x; OF_y; time_constant');
disp(params);





% シミュレーションを実行
Simu_Val_optimized = [params(1:3); const1; params(4:5)]; % const1の正しい更新方法
offset_value_optimized = params(6:7);
time_constant = params(8);




options = odeset('RelTol',1e-6,'AbsTol',1e-6);
[ta, Current] = ode45(@(t, Current) system_dynamics(t, Current, Simu_Val_optimized, offset_value_optimized,...
                                           const2, fdX, fR_BtoI, fP_BtoI,time_constant), tspan, Current0, options);

% 5%整定時間を求める。
for i = 4:6
    settling_time_index = find(abs(Current(:,i) - Current(end, i)) > abs(0-Current(end, i))*0.05 ,1,'last');
    settling_time = ta(settling_time_index);
    
    % 結果を表示
    disp(['整定時間: ', num2str(settling_time), ' 秒']);
end

% 遅れ時間
for i = 4:6
    settling_time_index = find(Current(:,i) > Current(end, i)*0.5 ,1);
    settling_time = ta(settling_time_index);
    
    % 結果を表示
    disp(['遅れ時間: ', num2str(settling_time), ' 秒']);
end

% オーバーシュート
for i = 4:6
    settling_time_index = max(Current(:,i)-Current(end, i))*100/(Current(end, i)-0);
    
    % 結果を表示
    disp(['オーバーシュート: ', num2str(settling_time_index), ' %']);
end


% Current1 = Current;
% ta1 = ta;
% % 結果を格納
% save('Current1.mat', 'Current1', "ta1");

Current2 = Current;
ta2 = ta;
% 結果を格納
save('Current2.mat', 'Current2', "ta2");

%% 
% シミュレーション結果の取り出し

figure(11); clf; % figure(1)を作成またはクリア

% ロール角 (φ)
subplot(3, 1, 1); % 3行1列の1番目のsubplot
hold on;
plot(ta, Current(:,4)*180/pi(),'LineWidth', 1.2);
hold off;
xlabel('Time [s]','FontSize',13);
ylabel('Roll Angle [degree]','FontSize',13);
legend('Simulation Data','FontSize',10); 

grid on;

% ピッチ角 (θ)
subplot(3, 1, 2); % 3行1列の2番目のsubplot
hold on;
plot(ta, Current(:,5)*180/pi(),'LineWidth', 1.2);
hold off;
xlabel('Time [s]','FontSize',13);
ylabel('Pitch Angle [degree]','FontSize',13);
legend('Simulation Data','FontSize',10); 

grid on;

% ヨー角 (ψ)
subplot(3, 1, 3); % 3行1列の3番目のsubplot
hold on;
plot(ta, Current(:,6)*180/pi(),'LineWidth', 1.2);
hold off;
xlabel('Time [s]','FontSize',13);
ylabel(' Yaw Angle [degree]','FontSize',13);
legend('Simulation Data','FontSize',10); 

grid on;



% figure(12); clf; hold on;
% 
% subplot(3,1,1);
% plot(ta, Current(:,10));
% title('dPhi (dPhi)');
% xlabel('時間 (秒)');
% ylabel('角速度 (rad/s)');
% 
% subplot(3,1,2);
% plot(ta, Current(:,11));
% title('dTheta (dTheta)');
% xlabel('時間 (秒)');
% ylabel('角速度 (rad/s)');
% 
% subplot(3,1,3);
% plot(ta, Current(:,12));
% title('dPsi (dPsi)');
% xlabel('時間 (秒)');
% ylabel('角速度 (rad/s)');
% 
% figure(13); clf; hold on;
% subplot(3,1,1);
% plot(ta, Current(:,1));
% title('位置(x)');
% xlabel('時間 (秒)');
% ylabel(' 位置(m)');
% 
% subplot(3,1,2);
% plot(ta, Current(:,2));
% title('位置(y)');
% xlabel('時間 (秒)');
% ylabel(' 位置(m)');
% 
% subplot(3,1,3);
% plot(ta, Current(:,3));
% title('位置(z)');
% xlabel('時間 (秒)');
% ylabel(' 位置(m)');
% 
% figure(14); clf; hold on;
% 
% subplot(3,1,1);
% plot(ta, Current(:,7));
% title('速度 (dx)');
% xlabel('時間 (秒)');
% ylabel('速度 (m/s)');
% 
% subplot(3,1,2);
% plot(ta, Current(:,8));
% title('速度 (dy)');
% xlabel('時間 (秒)');
% ylabel('速度 (m/s)');
% 
% subplot(3,1,3);
% plot(ta, Current(:,9));
% title('速度 (dz)');
% xlabel('時間 (秒)');
% ylabel('速度 (m/s)');

toc;

% system_dynamics
function dCurrent = system_dynamics(t, Current, Simu_Val,offset_value, const2, fdX, fR_BtoI, fP_BtoI,time_constant)
    Eta = Current(4:6);
    dEta = Current(10:12);
    X_I = Current(1:12);

    % 各種変換行列を作る
    R_BtoI = fR_BtoI(Eta);
    P_BtoI = fP_BtoI(Eta);

    etad=[pi*30/180;pi*30/180;pi*30/180];  %目標角
    % etad=[rand;rand;rand]*2; 

    kp = [20; 32; 70]; % 角度制御の比例ゲイ
    kd = [12; 12; 60 ]; % 角度制御の微分ゲイン

    % kp = [10; 32; 30]; % 角度制御の比例ゲイ
    % kd = [12; 12; 10 ]; % 角度制御の微分ゲイン

    Eta_error = etad - Eta;
    dEta_error = dEta; % 定義と符号逆

    % 100Hzで制御する。
    persistent cot_time u_log; % persistent 変数の宣言
    if isempty(cot_time)
        cot_time = 0;
        u_log =  kp .* (Eta_error) - kd .* dEta_error;
    end
      
      tf = floor(t * 100);
      if (tf-cot_time) > 0
          u0 =  kp .* (Eta_error) - kd .* dEta_error;
          u = u_log + 0.01*(u0 - u_log)/time_constant;
          u_log = u;
      else
          u = u_log;
      end
      cot_time = tf;
      
    
    Th = 800;


    motor_val = [(Th - u(1) + u(2) - u(3));
                 (Th - u(1) - u(2) + u(3));
                 (Th + u(1) + u(2) + u(3));
                 (Th + u(1) - u(2) - u(3))];
    for i =1:4
        if motor_val(i) > 900
            disp ('モーター出力限界')
        end
    end
    
    input_sim = zeros(1, 4); % Corrected initialization: 1 row, 4 columns
    
    for i = 1:4 % Corrected loop range: should iterate through all 4 motor values
        input_sim(i) = (motor_val(i) - 551) * 100 / (1023 - 551);
    end
    
    % disp(input_sim)
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

    % disp(motor_val_filtered)


    motor_torq_roll = sum((input_sim .* [Ly_Right Ly_Right Ly_Left Ly_Left])' .* [-1 -1 1 1]' *0.04438);
    motor_torq_pitch = sum((input_sim .* [Lx_Rear Lx_Front Lx_Rear Lx_Front])' .* [1 -1 1 -1]' *0.04438);
    motor_torq_yaw = sum((input_sim .* [L1 L2 L3 L4])' .* [-1 1 1 -1]' * 0.002069627691412);
    motor_torq = [motor_torq_roll; motor_torq_pitch; motor_torq_yaw];



    F_b = [0; 0; 1] * sum(input_sim) * 0.04438; 
    Tau_b = motor_torq;

    F_I = R_BtoI * F_b;
    Tau_I = P_BtoI * Tau_b;
    Q_I = [F_I; Tau_I];

    vars = {[X_I; Q_I; Simu_Val]};
    fdx_val = fdX(vars{:});
    dCurrent = fdx_val;
end