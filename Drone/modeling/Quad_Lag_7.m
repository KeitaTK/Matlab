%% 
% 損失を考慮した運動方程式を求める。
% 
% D = 0 にすると損失なしになる。
% 
% 慣性座標系で考える。

clear;
% 位置
% syms x_b y_b z_b real;                  % _bは機体座標系
syms dx_b dy_b dz_b real;                %速度
syms x_I y_I z_I real;                  % _bは機体座標系
syms dx_I dy_I dz_I real;                %速度
syms x_I(t) y_I(t) z_I(t)                 % _Iは慣性座標系
syms phi theta psi real;                 % 一般座標系から見た機体の傾き(機体座標系と初期値の慣性座標系は一致しているものとする)
syms dphi dtheta dpsi real;
syms m g real; 
syms J_xx J_yy J_zz real;                     % 慣性モーメント
syms F_x_I F_y_I F_z_I real;                        % 機体に働く並進方向の力
syms tau_x_I tau_y_I tau_z_I real;                  % 機体に働くモーメント
syms t real;
syms mu_1 mu_2 real;
%% 
% 一般化座標と一般化力を定義

X_I = [x_I; y_I; z_I];
Eta = [phi; theta; psi];
q =[X_I; Eta];       % 一般化座標

dX_I = [dx_I; dy_I; dz_I];
dEta = [dphi; dtheta; dpsi];
dq = [dX_I; dEta];    % 一般化速度

F_I = [F_x_I; F_y_I; F_z_I];
Tau_I = [tau_x_I; tau_y_I; tau_z_I];
Q = [F_I; Tau_I];

J = [J_xx 0 0;
     0 J_yy 0;
     0 0 J_zz ]
%% 
% 機体の角速度ベクトル ω を求める。

load("Omega.mat", "fOmega");       % 関数ハンドルをロード
Omega = fOmega([Eta; dEta]);
%% 
% 運動エネルギーを求める。並進と回転の合計。

W_1 = 0.5 * m * (dX_I)' * dX_I;
W_2 = 0.5 * (Omega') * J * Omega;
W = W_1 + W_2;
%% 
% 位置エネルギーを求める。

U = m * g * z_I;  
%% 
% 損失エネルギーを求める。D = 0　にすると、損失なし。

D = (0.5 * mu_1 * (dX_I') * dX_I) +(0.5 * mu_2 * (Omega') * Omega);
% D = 0;
%% 
% ラグラジアンを計算

L = W - U;
L = simplify(L);
%% 
% オイラー・ラグランジュ方程式の左辺を計算
% 
% M は慣性行列，CoriGrav は遠心力とコリオリ力、ポテンシャルから受ける力の和に損失項を足したもの

dL_dq_dot = jacobian(L, dq);
dL_dq     = jacobian(L, q);
dD_dq_dot = jacobian(D, dq);

M = jacobian(dL_dq_dot, dq);
M = expand(M);

h      = simplify(jacobian(dL_dq_dot, q) * dq);
g_term = simplify(dL_dq.');        % ← 変数名を変更

CoriGrav = jacobian(dL_dq_dot, q) * dq - dL_dq.' + dD_dq_dot.';
CoriGrav = simplify(CoriGrav);
%% 
% 作った関数を保存。

X  = [X_I; Eta; dX_I; dEta];
dX = [dq; M \ (-CoriGrav + Q)];
dX = simplify(dX);

fdX = matlabFunction(dX, 'Vars', {[
    x_I,  y_I,  z_I,  phi,  theta, psi, ...
    dx_I, dy_I, dz_I, dphi, dtheta, dpsi, ...
    F_x_I, F_y_I, F_z_I, tau_x_I, tau_y_I, tau_z_I, ...
    J_xx, J_yy, J_zz, m, g, mu_1, mu_2
]});
save("dX.mat", "fdX");