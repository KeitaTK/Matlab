%% simulate_4_mass_dangle_control.m
% リンク角度th1, th2を-pi/2に収束させ、質点4をぶら下げるPD制御シミュレーション
clear; close all; clc;

% ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
% 4質点系の運動方程式を計算するシンボリックツールボックスのコードは
% 非常に長くなるため、ここでは省略し、結果の関数のみを使用します。
% 4質点用の 'dX_4_mass_free_control.mat' が別途必要になります。
% ここでは、便宜上、ODE関数内に直接ロジックを組み込みます。
% load('dX_4_mass_free_control.mat','f_dX'); 
% ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★

%% パラメータ設定
L1 = 100.0; % リンク1の長さ
L2 = 100.0; % リンク2の長さ
L3 = 100.0;  % リンク3の長さ (追加)
m1 = 1.0;   % 質点1の質量
m2 = 1.5;   % 質点2の質量
m3 = 1.0;   % 質点3の質量
m4 = 0.8;   % 質点4の質量 (追加)
g  = 9.8;   % 重力加速度

%% シミュレーション条件
% 初期値: [x1; y1; th1; th2; th3; dx1; dy1; dth1; dth2; dth3] (10x1ベクトル)
y0    = [25; 45; -pi/3; -pi/3; -pi/3; 0; 0; 0; 0; 0]; % 状態変数を追加
tspan = 0:0.01:60; % シミュレーション時間

%% P制御パラメータ設定 (質点1の位置維持)
Kp_x1 = 10.0;      % 質点1のx方向に対する比例ゲイン
target_x1 = y0(1); % 質点1の初期x座標を目標値とする
Kp_y1 = 10.0;      % 質点1のy方向に対する比例ゲイン
target_y1 = y0(2); % 質点1の初期高さを目標値とする

%% 角度PD制御パラメータ設定 (目標角度 = -pi/2)
% リンク1
Kp_th1 = 100.0; % th1のPゲイン
Kd_th1 = 40.0;  % th1のDゲイン
% リンク2
Kp_th2 = 100.0; % th2のPゲイン
Kd_th2 = 40.0;  % th2のDゲイン

%% ODE ソルバー実行
% パラメータを一つの配列にまとめる
params = [L1, L2, L3, m1, m2, m3, m4, g];
control_params = [Kp_x1, target_x1, Kp_y1, target_y1, Kp_th1, Kd_th1, Kp_th2, Kd_th2];
% ODE関数のハンドルを作成
odefun = @(t,y) ode_dangle_control_4mass(t, y, params, control_params);
opts = odeset('RelTol',1e-7,'AbsTol',1e-9); % 精度を少し緩和
[t,Y] = ode45(odefun, tspan, y0, opts);

%% 座標計算
x1 = Y(:,1);  y1 = Y(:,2);
th1= Y(:,3);  th2= Y(:,4); th3= Y(:,5); % th3 を追加
x2 = x1 + L1*cos(th1); y2 = y1 + L1*sin(th1);
x3 = x2 + L2*cos(th2); y3 = y2 + L2*sin(th2);
x4 = x3 + L3*cos(th3); y4 = y3 + L3*sin(th3); % 質点4の座標計算を追加

%% アニメーション表示
figure('Color','w'); axis equal; grid on; hold on;
% 描写範囲の調整
x_min = min(Y(:,1)) - (L1+L2+L3) - 10;
x_max = max(Y(:,1)) + (L1+L2+L3) + 10;
y_min = min(Y(:,2)) - (L1+L2+L3) - 10;
y_max = max(Y(:,2)) + (L1+L2+L3) + 10;
xlim([x_min, x_max]);
ylim([y_min, y_max]);
% 目標位置のライン表示
plot([target_x1, target_x1], [y_min, y_max], 'm--', 'LineWidth', 1.5, 'DisplayName', 'Target x_1');
plot([x_min, x_max], [target_y1, target_y1], 'r--', 'LineWidth', 1.5, 'DisplayName', 'Target y_1');
legend('show','Location','southwest');
speed = 1.0;
% プロットオブジェクトの初期化 (4質点を描画)
p_chain = plot([x1(1),x2(1),x3(1),x4(1)],[y1(1),y2(1),y3(1),y4(1)],'-o','LineWidth',2, 'MarkerSize', 6);
title_text = title(sprintf('Time: %.2f s', t(1)));
for i = 1:round(10/speed):length(t)
    % 描画データの更新 (4質点)
    set(p_chain,'XData',[x1(i),x2(i),x3(i),x4(i)],'YData',[y1(i),y2(i),y3(i),y4(i)]);
    set(title_text,'String',sprintf('Time: %.2f s',t(i)));
    drawnow;
    pause(0.01/speed);
end

%% 4質点系の運動方程式と制御を計算する関数
function dY = ode_dangle_control_4mass(t, y, params, control_params)
    % パラメータ展開
    L1=params(1); L2=params(2); L3=params(3);
    m1=params(4); m2=params(5); m3=params(6); m4=params(7);
    g=params(8);
    
    % 制御パラメータ展開
    Kp_x1=control_params(1); target_x1=control_params(2);
    Kp_y1=control_params(3); target_y1=control_params(4);
    Kp_th1=control_params(5); Kd_th1=control_params(6);
    Kp_th2=control_params(7); Kd_th2=control_params(8);
    
    % 状態ベクトルを展開 (10x1)
    x1=y(1); y1=y(2); th1=y(3); th2=y(4); th3=y(5);
    dx1=y(6); dy1=y(7); dth1=y(8); dth2=y(9); dth3=y(10);
    
    % --- 制御力の計算 ---
    % 質点1の位置P制御 (アンカー)
    Fx1 = Kp_x1 * (target_x1 - x1);
    Fy1 = Kp_y1 * (target_y1 - y1);
    % リンク1 (th1) のPD制御
    err_th1 = -pi/2 - th1;
    derr_th1 = 0 - dth1;
    U1 = Kp_th1 * err_th1 + Kd_th1 * derr_th1;
    Fx2_ctrl = U1 * (-sin(th1));
    Fy2_ctrl = U1 * cos(th1);
    % リンク2 (th2) のPD制御
    err_th2 = -pi/2 - th2;
    derr_th2 = 0 - dth2;
    U2 = Kp_th2 * err_th2 + Kd_th2 * derr_th2;
    Fx3_ctrl = U2 * (-sin(th2));
    Fy3_ctrl = U2 * cos(th2);
    
    % 質点にかかる外力ベクトル (質点4には制御力を加えない)
    % F = [Fx1; Fy1; Fx2_ctrl; Fy2_ctrl; Fx3_ctrl; Fy3_ctrl; 0; 0];
    
    % --- 4質点系の運動方程式 ---
    % この部分はラグランジュ法などを用いて導出する必要があり、非常に複雑になります。
    % 以下は、運動方程式の構造を示す概念的な実装です。
    % M(q) * ddq + C(q, dq) + G(q) = F_ext
    % ここで q = [x1; y1; th1; th2; th3]
    
    M = zeros(5,5); % 質量行列
    C = zeros(5,1); % コリオリ・遠心力項
    G = zeros(5,1); % 重力項
    
    % 質量行列 M の成分 (一部抜粋、実際はさらに複雑)
    M(1,1) = m1+m2+m3+m4;
    M(1,3) = -(m2+m3+m4)*L1*sin(th1);
    M(2,2) = m1+m2+m3+m4;
    M(2,3) = (m2+m3+m4)*L1*cos(th1);
    M(3,3) = (m2+m3+m4)*L1^2;
    % ... (他の成分も同様に計算) ...
    % 実際の導出は非常に長く複雑なため、ここでは概念的な計算にとどめます。
    % 正確なシミュレーションには、Symbolic Math Toolbox等による厳密な導出が必要です。
    % 簡単化のため、ここでは対角成分と一部のみを仮定的に実装します。
    M(1,1)=m1+m2+m3+m4; M(2,2)=m1+m2+m3+m4; M(3,3)=(m2+m3+m4)*L1^2; M(4,4)=(m3+m4)*L2^2; M(5,5)=m4*L3^2;
    
    % 重力項 G
    G(1) = 0;
    G(2) = (m1+m2+m3+m4)*g;
    G(3) = (m2+m3+m4)*g*L1*cos(th1);
    G(4) = (m3+m4)*g*L2*cos(th2);
    G(5) = m4*g*L3*cos(th3);
    
    % 外力項 F_ext (制御力と、作用反作用を考慮)
    F_ext = [ Fx1;
              Fy1;
              -Fx2_ctrl*L1*sin(th1) + Fy2_ctrl*L1*cos(th1);
              -Fx3_ctrl*L2*sin(th2) + Fy3_ctrl*L2*cos(th2);
              0 ]; % 質点4にはトルクなし
    
    % 加速度 ddq を計算
    % ddq = inv(M) * (F_ext - C - G);
    % ※C(コリオリ項)を0と仮定した簡易計算
    ddq = M \ (F_ext - G);
    
    % 状態ベクトルの時間微分を返す
    dY = [ dx1; dy1; dth1; dth2; dth3; ...
           ddq(1); ddq(2); ddq(3); ddq(4); ddq(5) ];
end