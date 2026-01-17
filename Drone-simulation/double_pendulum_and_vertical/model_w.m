%% double_pendulum_Lag.m
% 二重振り子のラグランジュから運動方程式を導出し、
% 生成された状態方程式をMATファイルとして保存
% 同フォルダ内の simulate_double_pendulum.m で利用

clear;
%% シンボリック変数定義
syms theta1 theta2 real                     % 角度
syms dtheta1 dtheta2 real                   % 角速度
syms L1 L2 m1 m2 g real                     % パラメータ
syms t real

% 一般化座標と速度ベクトル
q     = [theta1; theta2];
dq    = [dtheta1; dtheta2];

%% 質点の位置ベクトル
% 第1質点
x1 = L1*sin(theta1);
y1 = -L1*cos(theta1);
% 第2質点
x2 = x1 + L2*sin(theta2);
y2 = y1 - L2*cos(theta2);

%% 運動エネルギー
T1 = simplify( 0.5*m1*(diff(x1,theta1)*dtheta1)^2 ...
              + 0.5*m1*(diff(y1,theta1)*dtheta1)^2 );
T2 = simplify( 0.5*m2*(diff(x2,theta1)*dtheta1 + diff(x2,theta2)*dtheta2)^2 ...
              + 0.5*m2*(diff(y2,theta1)*dtheta1 + diff(y2,theta2)*dtheta2)^2 );
T  = simplify(T1 + T2);

%% ポテンシャルエネルギー
V1 = m1*g*y1;
V2 = m2*g*y2;
V  = simplify(V1 + V2);

%% ラグランジアン
Lagr = simplify(T - V);

%% 減衰なし（D=0）
D = sym(0);

%% ラグランジュ方程式の構築
dL_dq    = jacobian(Lagr, q).';
dL_ddq   = jacobian(Lagr, dq).';
ddt_dL_ddq = jacobian(dL_ddq, q)*dq + jacobian(dL_ddq, dq)*[0;0];  
% ※ accel 項は M^(-1)*(… ) で後出し

% 質点間カップリング含む質量行列 M(q)
M = simplify( jacobian(dL_ddq, dq) );

% コリオリ・重力項
CG = simplify( ddt_dL_ddq - dL_dq );

% 状態方程式 d²q = M^{-1} * (-CG)
ddq = simplify( M \ (-CG) );

%% 状態ベクトル dX = [dq; ddq]
dX_sym = [dq; ddq];

%% ファンクション生成＆保存
all_vars = [theta1, theta2, dtheta1, dtheta2, L1, L2, m1, m2, g];
f_dX = matlabFunction(dX_sym, 'Vars', all_vars);
save('dX_double_pendulum.mat', 'f_dX');

disp('二重振り子状態方程式を dX_double_pendulum.mat に保存しました');
