clear; clc;

% --- パラメータ ---
g = 9.81;         % 重力加速度
L = 1.0;          % 紐の長さ
omega = sqrt(g/L);
A = 0.5;          % 外乱の振幅
dt = 0.01;
T = 20;
N = T/dt;
xref = 1.0;

% PID ゲイン（必要に応じて調整）
Kp = 5; Kd = 4; Ki = 0.2;
K_theta = 10;  % θ̂ に対する抑制ゲイン

% オブザーバゲイン（θ, θ̇ 推定）
L_obs = [30; 500];  % 2次オブザーバのゲイン

% --- 初期化 ---
x1 = zeros(4, N);  % 通常制御（補償なし）
x2 = zeros(4, N);  % θ̂補償あり制御
z_obs = zeros(2, N);  % オブザーバ状態 [θ̂; θ̇̂]

e_int1 = 0; e_int2 = 0;
x_log1 = zeros(1,N-1); theta_log1 = zeros(1,N-1);
x_log2 = zeros(1,N-1); theta_log2 = zeros(1,N-1);

% --- メインループ ---
for k = 1:N-1
    % 外乱（吊り荷による周期正弦外乱）
    d = -A * sin(omega * (k*dt));

    % ========== 通常PID ==========
    e1 = xref - x1(1,k); de1 = -x1(2,k); e_int1 = e_int1 + e1*dt;
    u1 = Kp*e1 + Kd*de1 + Ki*e_int1;

    dx1 = x1(2,k);
    dx2 = u1 + d;
    dx3 = x1(4,k);
    dx4 = -g/L * x1(3,k) - (1/L) * u1;
    x1(:,k+1) = x1(:,k) + dt * [dx1; dx2; dx3; dx4];

    x_log1(k) = x1(1,k); theta_log1(k) = x1(3,k);

    % ========== θ̂補償制御 ==========
    e2 = xref - x2(1,k); de2 = -x2(2,k); e_int2 = e_int2 + e2*dt;
    u_pid = Kp*e2 + Kd*de2 + Ki*e_int2;

    % θオブザーバ（観測 y = x2(1,k)）
    y = x2(1,k);

    % 観測誤差（観測値 - 推定値）
    est_error = y - z_obs(1,k);

    dz1 = z_obs(2,k) + L_obs(1) * est_error;
    dz2 = -g/L * z_obs(1,k) - (1/L) * u_pid + L_obs(2) * est_error;
    z_obs(:,k+1) = z_obs(:,k) + dt * [dz1; dz2];

    u2 = u_pid - K_theta * z_obs(1,k);  % θ̂ による補償

    dx1 = x2(2,k);
    dx2 = u2 + d;
    dx3 = x2(4,k);
    dx4 = -g/L * x2(3,k) - (1/L) * u2;
    x2(:,k+1) = x2(:,k) + dt * [dx1; dx2; dx3; dx4];

    x_log2(k) = x2(1,k); theta_log2(k) = x2(3,k);
end

t = 0:dt:(N-2)*dt;

% --- グラフ表示 ---
figure('Position', [100 100 800 600]);

subplot(2,1,1);
plot(t, x_log1, 'k--', t, x_log2, 'b', 'LineWidth', 1.3);
xlabel('時間 [s]'); ylabel('位置 x [m]');
title('位置制御の比較');
legend('補償なし', 'θ̂補償あり');
grid on;

subplot(2,1,2);
plot(t, theta_log1, 'k--', t, theta_log2, 'b', 'LineWidth', 1.3);
xlabel('時間 [s]'); ylabel('吊り荷の角度 \theta [rad]');
title('吊り荷の揺れの比較');
legend('補償なし', 'θ̂補償あり');
grid on;
