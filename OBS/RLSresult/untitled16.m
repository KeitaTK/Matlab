clear; clc; close all;

%% ===== 設定 =====
csv_file   = "OBSV_data_00000404.csv";
f_target   = 0.59;              % 設定周波数 [Hz]
omega      = 2*pi*f_target;     % [rad/s]
dt_predict = 0.01;              % 予測時間 [s]

%% ===== CSV 読み込み =====
T = readtable(csv_file);

time_sec = (T.TimeUS - T.TimeUS(1)) * 1e-6;

AX  = T.AX;
BX  = T.BX;
CX  = T.CX;
PLX = T.PLX;

N = height(T);

%% ===== 観測振幅・観測位相 =====
A_obs   = sqrt(AX.^2 + BX.^2);
phi_obs = atan2(-BX, AX);        % [rad]
phi_obs = unwrap(phi_obs);       % 連続化
phi_deg = rad2deg(phi_obs);      % 表示用 [deg]

%% ===== 理想位相 =====
phi_ref = omega * time_sec;      % [rad]

%% ===== 位相誤差 =====
phase_error = wrapToPi(phi_ref - phi_obs);   % [rad]
phase_error_deg = rad2deg(phase_error);

%% ===== モデル外力 & 予測外力 =====
phi_now  = omega * time_sec;
phi_pred = omega * (time_sec + dt_predict);

F_model = AX .* sin(phi_now)  + BX .* cos(phi_now)  + CX;
F_pred  = AX .* sin(phi_pred) + BX .* cos(phi_pred) + CX;

%% ===== プロット =====
figure('Units','normalized','Position',[0.1 0.1 0.9 0.85])

% --- 外力（モデル & 予測） ---
subplot(4,1,1)
plot(time_sec, F_model,'b--','LineWidth',1.2); hold on;
%plot(time_sec, F_pred,'r:','LineWidth',1.4);
ylabel('Force [N]')
legend('Model (AX,BX,CX)','Predicted F_{pred}')
title('Model and Predicted Force')
grid on

% --- 観測振幅 ---
subplot(4,1,2)
plot(time_sec, A_obs,'m','LineWidth',1.4)
ylabel('Amplitude A_{obs}')
title('Observed Amplitude')
grid on

% --- 観測位相 ---
subplot(4,1,3)
plot(time_sec, phi_deg,'g','LineWidth',1.4)
ylabel('Phase \phi_{obs} [deg]')
title('Observed Phase')
grid on

% --- 位相誤差 ---
subplot(4,1,4)
plot(time_sec, phase_error_deg,'k','LineWidth',1.4)
ylabel('Phase Error [deg]')
xlabel('Time [s]')
title('Phase Error  e_\phi = \omega t - \phi_{obs}')
grid on

disp('モデル外力・観測振幅・観測位相・位相誤差の統合プロットが完了しました');


clear; clc; close all;

%% ===== 設定 =====
csv_file   = "OBSV_data_00000404.csv";
f_target   = 0.59;              % 設定周波数 [Hz]
omega      = 2*pi*f_target;     % [rad/s]
dt_predict = 0.01;              % 予測時間 [s]

%% ===== CSV 読み込み =====
T = readtable(csv_file);

time_sec = (T.TimeUS - T.TimeUS(1)) * 1e-6;
dt = mean(diff(time_sec));

AX  = T.AX;
BX  = T.BX;
CX  = T.CX;
PLX = T.PLX;

N = height(T);

%% ===== 観測振幅・観測位相 =====
A_obs   = sqrt(AX.^2 + BX.^2);
phi_obs = atan2(-BX, AX);      % [rad]
phi_obs = unwrap(phi_obs);     % 位相連続化
phi_deg = rad2deg(phi_obs);

%% ===== 理想位相 =====
phi_ref = omega * time_sec;

%% ===== 位相誤差（3種類）=====
phase_error_raw   = phi_ref - phi_obs;                 % 未wrap
phase_error_wrap  = wrapToPi(phase_error_raw);         % 制御用
phase_error_cont  = unwrap(phase_error_raw);           % 連続評価用

%% ===== 位相誤差速度（連続性指標）=====
phase_error_rate = [0; diff(phase_error_cont)] / dt;  % [rad/s]

%% ===== モデル外力 & 予測外力 =====
phi_now  = omega * time_sec;
phi_pred = omega * (time_sec + dt_predict);

F_model = AX .* sin(phi_now)  + BX .* cos(phi_now)  + CX;
F_pred  = AX .* sin(phi_pred) + BX .* cos(phi_pred) + CX;

%% ===== プロット =====
figure('Units','normalized','Position',[0.05 0.05 0.9 0.9])

% --- 外力 ---
subplot(5,1,1)
plot(time_sec, F_model,'b--','LineWidth',1.2); hold on;
%plot(time_sec, F_pred,'r:','LineWidth',1.4);
ylabel('Force [N]')
title('Model and Predicted Force')
legend('Model','Predicted')
grid on

% --- 観測振幅 ---
subplot(5,1,2)
plot(time_sec, A_obs,'m','LineWidth',1.4)
ylabel('A_{obs}')
title('Observed Amplitude')
grid on

% --- 観測位相 ---
subplot(5,1,3)
plot(time_sec, phi_deg,'g','LineWidth',1.4)
ylabel('\phi_{obs} [deg]')
title('Observed Phase (Unwrapped)')
grid on

% --- 位相誤差（連続） ---
subplot(5,1,4)
plot(time_sec, rad2deg(phase_error_cont),'k','LineWidth',1.4)
ylabel('e_\phi^{cont} [deg]')
title('Phase Error (Continuous)')
grid on

% --- 位相誤差速度 ---
subplot(5,1,5)
plot(time_sec, phase_error_rate,'r','LineWidth',1.2)
ylabel('d e_\phi / dt [rad/s]')
xlabel('Time [s]')
title('Phase Error Rate (Continuity Indicator)')
grid on

disp('位相誤差の連続性評価が可能な解析が完了しました');
