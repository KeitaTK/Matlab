clear;
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
Pos0 = [X0; dX0];

eta0 = [40*pi/180; 20*pi/180; 10*pi/180];   % 角度
deta0 = [0; 0; 0];   % 角速度
Att0 = [eta0; deta0];

Current0 = [X0; eta0; dX0; deta0];
%% 
% シミュレーションの環境設定

etad=[0;0;pi*0/180];  %目標角

kp = [5; 16; 50]; % 角度制御の比例ゲイン
kd = [6; 6; 5]; % 角度制御の微分ゲイン

setting_val = [etad; kp; kd];

tspan = 0:0.01:10;      % シミュレーション時間
%% 
% 定数に値を入れる 

J_xx = 0.01; J_yy = 0.01; J_zz = 0.01; 
m = 0.7; g = 9.81; mu_1 = 0.1; mu_2 = 0.01; 
const = [J_xx; J_yy; J_zz; m; g; mu_1; mu_2;];

% % 外力 (例: すべて0)
% F_b = [0; 0; 0];
% Tau_b = [0; 0; 0];
% Q_b = [F_b; Tau_b];
%% 
% ode45でシミュレーション

options = odeset('RelTol',1e-6,'AbsTol',1e-6);
[ta, Current] = ode45(@(t, Current) system_dynamics(t, Current, const, setting_val, fdX, fR_BtoI, fOmega, fP_BtoI, fP_ItoB), tspan, Current0,options);

%% 
% シミュレーション結果の取り出し

X_b_sim = Current(:, 1:3);
Eta_sim = Current(:, 4:6);
dX_b_sim = Current(:, 7:9);
dEta_sim = Current(:, 10:12);

figure(1); clf; hold on;
subplot(3,1,1);
plot(ta, Current(:,4));
title('ロール角 (φ)');
xlabel('時間 (秒)');
ylabel('角度 (rad)');
 
subplot(3,1,2);
plot(ta, Current(:,5));
title('ピッチ角 (θ)');
xlabel('時間 (秒)');
ylabel('角度 (rad)');
 
subplot(3,1,3);
plot(ta, Current(:,6));
title('ヨー角 (Ψ)');
xlabel('時間 (秒)');
ylabel('角度 (rad)');

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

% system_dynamics
function dCurrent = system_dynamics(~, Current, const, setting_val, fdX, fR_BtoI, fOmega, fP_BtoI, fP_ItoB)
    X_I = Current(1:3);
    Eta_I = Current(4:6);
    dX_I = Current(7:9);
    dEta_I = Current(10:12);
    X_I = Current;
    % F_b = Q_b(1:3);
    % Tau_b = Q_b(4:6);
    etad_b = setting_val(1:3);
    kp = setting_val(4:6);
    kd = setting_val(7:9);

    % 各種変換行列を作る
    R_BtoI = fR_BtoI(Eta_I);
    Omega = fOmega([Eta_I; dEta_I]);
    P_BtoI = fP_BtoI(Eta_I);
    P_ItoB = fP_ItoB(Eta_I);

    % 慣性座標系の角度と角速度を機体座標系に変換
    Eta_b =  P_ItoB * Eta_I;
    dEta_b = P_ItoB * dEta_I;

    %姿勢のPID制御器を実装
    input_motor = -kp .* (Eta_b-etad_b) - kd .* dEta_b
    % 551~1023の間での実機の入力にプラスマイナスする値
    input_sim = input_motor*100/(1023-551);
    motor_torq = 4*diag([0.1425,0.090,1])*(diag([0.022191707538092,0.022191707538092,3.533461241187303e-04]) * input_sim);  % 重心からの距離(m) * モーターの生み出す力(N)
    F_b = [0; 0; 1] * 80 * 0.022191707538092 * 4;
    Tau_b = motor_torq;

    F_I = R_BtoI * F_b;
    Tau_I = P_BtoI * Tau_b;
    Q_I = [F_I; Tau_I];

    vars = {[X_I; Q_I; const]};
    fdx_val = fdX(vars{:});
    dCurrent = fdx_val;
end