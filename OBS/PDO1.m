clear; clc;

% --- パラメータ設定 ---
g = 9.81;
L = 1.0;
omega = sqrt(g/L);
A = 0.5;
dt = 0.01;
T = 20;
N = T/dt;

% PIDゲイン
Kp = 5;
Kd = 4;
Ki = 0.2;

% PDOゲイン
l1 = 20; l2 = 100; l3 = 200; l4 = 200;

xref = 1.0;

% 初期化（補償なし）
x1 = zeros(4, N);
e_int1 = 0;
u_log1 = zeros(1, N-1);
x_rec1 = zeros(1, N-1);
theta_rec1 = zeros(1, N-1);

% 初期化（補償あり）
x2 = zeros(4, N);
x_hat = zeros(4, N);
e_int2 = 0;
u_log2 = zeros(1, N-1);
x_rec2 = zeros(1, N-1);
theta_rec2 = zeros(1, N-1);
d_est_log = zeros(1, N-1);
d_real_log = zeros(1, N-1);

% --- メインループ ---
for k = 1:N-1
    % --- 共通外乱 ---
    f_swing = -A * sin(omega * (k*dt));
    d_real_log(k) = f_swing;

    % === ① 通常PID制御 ===
    e1 = xref - x1(1,k);
    e_dot1 = -x1(2,k);
    e_int1 = e_int1 + e1*dt;
    u1 = Kp*e1 + Kd*e_dot1 + Ki*e_int1;

    dx1 = x1(2,k);
    dx2 = u1 + f_swing;
    dx3 = x1(4,k);
    dx4 = -g/L * x1(3,k) - (1/L) * u1;
    x1(:,k+1) = x1(:,k) + dt * [dx1; dx2; dx3; dx4];

    u_log1(k) = u1;
    x_rec1(k) = x1(1,k);
    theta_rec1(k) = x1(3,k);

    % === ② PDO補償制御 ===
    e2 = xref - x2(1,k);
    e_dot2 = -x2(2,k);
    e_int2 = e_int2 + e2*dt;
    u2_raw = Kp*e2 + Kd*e_dot2 + Ki*e_int2;

    % PDO推定
    y = x2(1,k); y_hat = x_hat(1,k); ey = y - y_hat;
    dxhat1 = x_hat(2,k) + l1*ey;
    dxhat2 = u2_raw + A*x_hat(3,k) + l2*ey;
    dxhat3 = omega * x_hat(4,k) + l3*ey;
    dxhat4 = -omega * x_hat(3,k) + l4*ey;
    x_hat(:,k+1) = x_hat(:,k) + dt * [dxhat1; dxhat2; dxhat3; dxhat4];

    d_est = A * x_hat(3,k);
    u2 = u2_raw - d_est;
    d_est_log(k) = d_est;

    dx1 = x2(2,k);
    dx2 = u2 + f_swing;
    dx3 = x2(4,k);
    dx4 = -g/L * x2(3,k) - (1/L) * u2;
    x2(:,k+1) = x2(:,k) + dt * [dx1; dx2; dx3; dx4];

    u_log2(k) = u2;
    x_rec2(k) = x2(1,k);
    theta_rec2(k) = x2(3,k);
end

t = 0:dt:(N-2)*dt;

% --- アニメーション ---
figure;
set(gcf,'Position',[100 100 1000 400])
for k = 1:20:N-1
    clf;
    subplot(1,2,1);
    hold on;
    drone1_x = x_rec1(k);
    drone1_y = 0;
    theta1 = theta_rec1(k);
    mass1_x = drone1_x + L*sin(theta1);
    mass1_y = drone1_y - L*cos(theta1);

    plot(drone1_x, drone1_y, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    plot([drone1_x, mass1_x], [drone1_y, mass1_y], 'k-', 'LineWidth', 2);
    plot(mass1_x, mass1_y, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    title('補償なし');
    axis equal; xlim([-1.5 2.5]); ylim([-1.5 0.5]); grid on;

    subplot(1,2,2);
    hold on;
    drone2_x = x_rec2(k);
    drone2_y = 0;
    theta2 = theta_rec2(k);
    mass2_x = drone2_x + L*sin(theta2);
    mass2_y = drone2_y - L*cos(theta2);

    plot(drone2_x, drone2_y, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    plot([drone2_x, mass2_x], [drone2_y, mass2_y], 'k-', 'LineWidth', 2);
    plot(mass2_x, mass2_y, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    title('PDO補償あり');
    axis equal; xlim([-1.5 2.5]); ylim([-1.5 0.5]); grid on;

    pause(0.01);
end
