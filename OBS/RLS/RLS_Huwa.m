clear; clc;

%% ▼ 設定
f_target = 0.6;         % 周期外乱周波数 [Hz]
omega = 2*pi*f_target;      % [rad/s]
lambda = 0.95;              % RLS 忘却係数
dt_predict = 0.01;          % 予測時間[s]
csv_file = '20250804_161747_EF.csv';
t_start = 18; t_end = 28;

%% ▼ データ読み込み
T = readtable(csv_file);
time_dt = datetime(T.Timestamp,'InputFormat','HH:mm.ss');
time_sec = seconds(time_dt - time_dt(1));
signal = T.Force_X_N;

dt = mean(diff(time_sec));
N = length(signal);

%% ▼ 参照信号生成（既知キャリア）
phi_ref = omega * time_sec;
phi_ref = phi_ref(:);  % 列ベクトル
Phi = [cos(phi_ref), sin(phi_ref)];

%% ▼ RLS 初期化
theta = zeros(2,1);  % [a;b]
P = 1000*eye(2);     % 共分散初期値
A_est = zeros(N,1);
phi_est = zeros(N,1);
a_est = zeros(N,1);
b_est = zeros(N,1);

%% ▼ RLSループ
for k = 1:N
    phi_k = Phi(k,:)';  
    y_k = signal(k);

    % RLSゲイン
    K = P*phi_k / (lambda + phi_k'*P*phi_k);

    % パラメータ更新
    theta = theta + K*(y_k - phi_k'*theta);

    % 共分散更新
    P = (P - K*phi_k'*P)/lambda;

    % 振幅・位相
    a = theta(1); b = theta(2);
    a_est(k) = a;
    b_est(k) = b;
    A_est(k) = sqrt(a^2 + b^2);    
    phi_est(k) = atan2(-b,a);
end

%% ▼ 抽出区間
idx = time_sec >= t_start & time_sec <= t_end;
t_seg = time_sec(idx);
y_seg = signal(idx);
a_seg = a_est(idx);
b_seg = b_est(idx);
A_seg = A_est(idx);
phi_seg = phi_est(idx);

%% ▼ 外乱推定（現在時刻）
d_obs = A_seg .* cos( omega .* t_seg + phi_seg );

%% ▼ 1ステップ未来予測（a,b成分を回転）
phi_pred = phi_seg + omega * dt_predict;   % 位相の回転
d_pred = A_seg .* cos( omega .* t_seg + phi_pred );

%% ▼ フィードフォワード補償
y_ff = y_seg - d_pred;

%% ▼ プロット
figure;
subplot(4,1,1)
plot(t_seg, y_seg,'k'); ylabel('Signal'); title('Original Signal');

subplot(4,1,2)
plot(t_seg, A_seg,'r'); ylabel('A(t)'); title('推定された振幅');

subplot(4,1,3)
plot(t_seg, rad2deg(phi_seg),'b'); ylabel('Phase [deg]'); title('推定された位相');

subplot(4,1,4)
plot(t_seg, y_seg,'k', t_seg, d_obs,'m--', t_seg, y_ff,'g');
legend('Original','外乱推定','補償'); xlabel('Time [s]');
title('1ステップ未来予測');
