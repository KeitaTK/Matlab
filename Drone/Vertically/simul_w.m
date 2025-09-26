%% simulate_double_pendulum.m
% 先に double_pendulum_Lag.m を実行して生成された
% dX_double_pendulum.mat を読み込み、シミュレーションとアニメーション

clear;
%% データ読み込み
load('dX_double_pendulum.mat','f_dX');

%% パラメータ設定
L1 = 1.0;
L2 = 1.0;
m1 = 1.0;
m2 = 1.0;
g  = 9.81;

%% 初期条件と時間設定
q0   = [pi/2; pi/2];
dq0  = [0; 0];
y0   = [q0; dq0];
tspan = [0, 20];

%% ODE関数ハンドル
odefun = @(t, y) f_dX( y(1), y(2), y(3), y(4), L1, L2, m1, m2, g );

%% 数値解
[T, Y] = ode45(odefun, tspan, y0);

%% アニメーション
figure('Name','Double Pendulum','NumberTitle','off');
for i = 1:5:length(T)
    clf;
    th1 = Y(i,1); th2 = Y(i,2);
    x1 = L1*sin(th1);  y1 = -L1*cos(th1);
    x2 = x1 + L2*sin(th2); y2 = y1 - L2*cos(th2);
    plot([0 x1 x2],[0 y1 y2],'-o','LineWidth',2);
    axis equal; axis([-2 2 -2.5 0.5]);
    title(sprintf('t = %.2f s', T(i)));
    drawnow;
end
disp('アニメーション終了');
