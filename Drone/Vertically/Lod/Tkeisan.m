%% simulate_4_mass_dangle_control_tension.m (アニメーション無し)
% 4質点系において、ラグランジュの未定乗数法を用いて張力を計算するPD制御シミュレーション
clear; close all; clc;

%% パラメータ設定
L1 = 100.0; % リンク1の長さ
L2 = 100.0; % リンク2の長さ
L3 = 100.0; % リンク3の長さ
m1 = 1.0;   % 質点1の質量
m2 = 1.5;   % 質点2の質量
m3 = 1.0;   % 質点3の質量
m4 = 0.8;   % 質点4の質量
g  = 9.8;   % 重力加速度

%% シミュレーション条件
% 初期角度からデカルト座標を計算
th1_0 = -pi/3; th2_0 = -pi/3; th3_0 = -pi/3;
x1_0 = 25; y1_0 = 45;
x2_0 = x1_0 + L1*cos(th1_0); y2_0 = y1_0 + L1*sin(th1_0);
x3_0 = x2_0 + L2*cos(th2_0); y3_0 = y2_0 + L2*sin(th2_0);
x4_0 = x3_0 + L3*cos(th3_0); y4_0 = y3_0 + L3*sin(th3_0);

% [x1; y1; x2; y2; x3; y3; x4; y4; dx1; dy1; ...; dx4; dy4] (16x1ベクトル)
y0 = [x1_0; y1_0; x2_0; y2_0; x3_0; y3_0; x4_0; y4_0; ...
      zeros(8,1)]; % 初期速度は0
tspan = 0:0.01:60; % シミュレーション時間

%% P制御パラメータ設定 (質点1の位置維持)
Kp_x1 = 10.0;
target_x1 = y0(1);
Kp_y1 = 10.0;
target_y1 = y0(2);

%% 角度PD制御パラメータ設定 (目標角度 = -pi/2)
Kp_th1 = 100.0; Kd_th1 = 40.0;  % リンク1
Kp_th2 = 100.0; Kd_th2 = 40.0;  % リンク2

%% ODE ソルバー実行
params = [L1, L2, L3, m1, m2, m3, m4, g];
control_params = [Kp_x1, target_x1, Kp_y1, target_y1, Kp_th1, Kd_th1, Kp_th2, Kd_th2];

odefun = @(t,y) ode_dangle_control_4mass_tension(t, y, params, control_params);
opts = odeset('RelTol',1e-6,'AbsTol',1e-8); % 精度を少し緩和
[t,Y] = ode45(odefun, tspan, y0, opts);

%% 座標と張力の計算
x1=Y(:,1); y1=Y(:,2); x2=Y(:,3); y2=Y(:,4);
x3=Y(:,5); y3=Y(:,6); x4=Y(:,7); y4=Y(:,8);

% 各時間ステップでの張力(λ)を再計算
lambdas = zeros(length(t), 3);
for i = 1:length(t)
    % ODE関数から2番目の戻り値としてλを取得
    [~, lambda_i] = ode_dangle_control_4mass_tension(t(i), Y(i,:)', params, control_params);
    lambdas(i,:) = lambda_i';
end
% 2*λが物理的な張力[N]に対応
T1 = 2*lambdas(:,1); % リンク1の張力
T2 = 2*lambdas(:,2); % リンク2の張力
T3 = 2*lambdas(:,3); % リンク3の張力

%% アニメーション表示を追加
figure('Name','4-Mass Pendulum Animation','Color','w');
axis equal; grid on; hold on;
% --- 修正版: 全質点の全時刻の座標から範囲を決定 ---
all_x = [x1(:); x2(:); x3(:); x4(:)];
all_y = [y1(:); y2(:); y3(:); y4(:)];
margin = 0.1 * (max([max(all_x)-min(all_x), max(all_y)-min(all_y)])); % 10%マージン
xlim_range = [min(all_x)-margin, max(all_x)+margin];
ylim_range = [min(all_y)-margin, max(all_y)+margin];
xlim(xlim_range);
ylim(ylim_range);
plot([target_x1, target_x1], ylim_range, 'm--', 'LineWidth', 1.5, 'DisplayName', 'Target x_1');
plot(xlim_range, [target_y1, target_y1], 'r--', 'LineWidth', 1.5, 'DisplayName', 'Target y_1');
legend('show','Location','southwest');

p_chain = plot([x1(1),x2(1),x3(1),x4(1)], [y1(1),y2(1),y3(1),y4(1)], '-o', ...
    'Color','b','LineWidth',2, 'MarkerSize',8, 'MarkerFaceColor','r');
title_text = title(sprintf('Time: %.2f s', t(1)));
speed = 5; % アニメーション速度
for i = 1:round(length(t)/(t(end)*speed*5)):length(t)
    set(p_chain,'XData',[x1(i),x2(i),x3(i),x4(i)],'YData',[y1(i),y2(i),y3(i),y4(i)]);
    set(title_text,'String',sprintf('Time: %.2f s',t(i)));
    drawnow;
end

%% 張力の時間変化をプロット
figure('Name','Tension Forces vs. Time','Color','w');
plot(t, T1, 'LineWidth', 1.5, 'DisplayName', 'Tension 1 (m1-m2)');
hold on; grid on;
plot(t, T2, 'LineWidth', 1.5, 'DisplayName', 'Tension 2 (m2-m3)');
plot(t, T3, 'LineWidth', 1.5, 'DisplayName', 'Tension 3 (m3-m4)');
xlabel('Time (s)');
ylabel('Tension (N)');
title('Tension in Each Link Over Time');
legend('show');
ylim([min(min([T1 T2 T3]))-10, max(max([T1 T2 T3]))+10]);

%% ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
%  ↓↓↓ ラグランジュの未定乗数法を実装した新しいODE関数 ↓↓↓
%% ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
function [dY, lambda] = ode_dangle_control_4mass_tension(t, y, params, control_params)
    % パラメータ展開
    L1=params(1); L2=params(2); L3=params(3);
    m1=params(4); m2=params(5); m3=params(6); m4=params(7);
    g=params(8);
    
    % 制御パラメータ展開
    Kp_x1=control_params(1); target_x1=control_params(2);
    Kp_y1=control_params(3); target_y1=control_params(4);
    Kp_th1=control_params(5); Kd_th1=control_params(6);
    Kp_th2=control_params(7); Kd_th2=control_params(8);
    
    % 状態ベクトルを展開 (16x1)
    q = y(1:8);  % 位置 [x1,y1,x2,y2,x3,y3,x4,y4]
    dq = y(9:16); % 速度 [dx1,dy1,dx2,dy2,...]
    
    x1=q(1); y1=q(2); x2=q(3); y2=q(4); x3=q(5); y3=q(6); x4=q(7); y4=q(8);
    dx1=dq(1); dy1=dq(2); dx2=dq(3); dy2=dq(4); dx3=dq(5); dy3=dq(6); dx4=dq(7); dy4=dq(8);
    % --- 制御力の計算 ---
    % 1. デカルト座標から角度と角速度を計算
    th1 = atan2(y2-y1, x2-x1);
    th2 = atan2(y3-y2, x3-x2);
    dth1 = ((dx2-dx1)*(y2-y1) - (x2-x1)*(dy2-dy1)) / L1^2;
    dth2 = ((dx3-dx2)*(y3-y2) - (x3-x2)*(dy3-dy2)) / L2^2;
    
    % 2. 質点1の位置P制御
    Fx1_p = Kp_x1 * (target_x1 - x1);
    Fy1_p = Kp_y1 * (target_y1 - y1);
    
    % 3. リンク1 (th1) のPD制御トルク U1
    err_th1 = -pi/2 - th1;
    derr_th1 = 0 - dth1;
    U1 = Kp_th1 * err_th1 + Kd_th1 * derr_th1;
    
    % 4. リンク2 (th2) のPD制御トルク U2
    err_th2 = -pi/2 - th2;
    derr_th2 = 0 - dth2;
    U2 = Kp_th2 * err_th2 + Kd_th2 * derr_th2;
    % 5. トルクを質点への力に変換 (作用・反作用)
    F1_ctrl_x = -U1/L1 * sin(th1); F1_ctrl_y = U1/L1 * cos(th1); % U1がm2に与える力
    F2_ctrl_x = -U2/L2 * sin(th2); F2_ctrl_y = U2/L2 * cos(th2); % U2がm3に与える力
    % 6. 全ての制御力と外力をベクトルにまとめる
    F_app = [ Fx1_p + F1_ctrl_x;          % Fx1
              Fy1_p + F1_ctrl_y;          % Fy1
              -F1_ctrl_x + F2_ctrl_x;     % Fx2
              -F1_ctrl_y + F2_ctrl_y;     % Fy2
              -F2_ctrl_x;                 % Fx3
              -F2_ctrl_y;                 % Fy3
              0;                          % Fx4
              0 ];                        % Fy4
    
    % 重力ベクトル G
    G = [0; m1*g; 0; m2*g; 0; m3*g; 0; m4*g];
    
    % --- 微分代数方程式(DAE)の構築 ---
    % [ M  J^T ] [ ddq   ] = [ F_app - G ]
    % [ J   0  ] [ lambda]   [ Qc        ]
    
    % 質量行列 M (8x8)
    M = diag([m1, m1, m2, m2, m3, m3, m4, m4]);
    
    % 拘束ヤコビアン J (3x8)
    J = zeros(3, 8);
    % リンク1 (m1-m2) の拘束: Jの1-4列目が対応
    J(1, 1:4) = 2 * [ (x1-x2), (y1-y2), (x2-x1), (y2-y1) ];
    % リンク2 (m2-m3) の拘束: Jの3-6列目が対応
    J(2, 3:6) = 2 * [ (x2-x3), (y2-y3), (x3-x2), (y3-y2) ];
    % リンク3 (m3-m4) の拘束: Jの5-8列目が対応
    J(3, 5:8) = 2 * [ (x3-x4), (y3-y4), (x4-x3), (y4-y3) ];
    
    % 加速度に関する項 Qc = -dJ/dt * dq
    Qc = zeros(3,1);
    Qc(1) = -2 * ((dx1-dx2)^2 + (dy1-dy2)^2);
    Qc(2) = -2 * ((dx2-dx3)^2 + (dy2-dy3)^2);
    Qc(3) = -2 * ((dx3-dx4)^2 + (dy3-dy4)^2);
    
    % DAEの左辺行列 A (11x11) と 右辺ベクトル b (11x1)
    A = [M, J'; J, zeros(3,3)];
    b = [F_app - G; Qc];
    
    % 連立方程式を解いて、加速度ddqと未定乗数λを求める
    sol = A \ b;
    
    ddq    = sol(1:8);   % 加速度ベクトル
    lambda = sol(9:11);  % 未定乗数ベクトル (張力に関連)
    
    % 状態ベクトルの時間微分を返す
    dY = [dq; ddq];
end