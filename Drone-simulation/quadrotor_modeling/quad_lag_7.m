%% 
% Quad_Lag_7.m
% ラグランジュの運動方程式によるクアッドコプター（ドローン）の運動モデル
% 損失項(mu_1, mu_2)や慣性モーメント(J_xx, J_yy, J_zz)を含む
% 慣性座標系での運動方程式を導出し、matファイルとして保存
% 
% 詳細はREADME.md参照

clear;
% 位置・速度・角度・慣性・力・トルク・損失係数のシンボリック変数定義
syms x_I y_I z_I real;
syms dx_I dy_I dz_I real;
syms phi theta psi real;
syms dphi dtheta dpsi real;
syms m g real;
syms J_xx J_yy J_zz real;
syms F_x_I F_y_I F_z_I real;
syms tau_x_I tau_y_I tau_z_I real;
syms mu_1 mu_2 real;
syms t real;

%% 一般化座標と速度
X_I   = [x_I; y_I; z_I];
Eta   = [phi; theta; psi];
dX_I  = [dx_I; dy_I; dz_I];
dEta  = [dphi; dtheta; dpsi];
dq    = [dX_I; dEta];
Q     = [F_x_I; F_y_I; F_z_I; tau_x_I; tau_y_I; tau_z_I];
J     = diag([J_xx, J_yy, J_zz]);

%% 角速度ベクトルの読み込みと生成
mat_dir = fullfile(pwd, 'mat_data');
load(fullfile(mat_dir, "Omega.mat"), "fOmega");
Omega = fOmega(phi, theta, psi, dphi, dtheta, dpsi);

%% エネルギー項
W_trans = 0.5 * m * (dX_I.' * dX_I);
W_rot   = 0.5 * Omega.' * J * Omega;
W       = W_trans + W_rot;
U       = m * g * z_I;  
D       = 0.5 * mu_1 * (dX_I.' * dX_I) + 0.5 * mu_2 * (Omega.' * Omega);

%% ラグランジアン
L = simplify(W - U);

%% オイラー–ラグランジュ方程式
dL_dq_dot = jacobian(L, dq);
dL_dq     = jacobian(L, [X_I; Eta]);
dD_dq_dot = jacobian(D, dq);

M        = expand(jacobian(dL_dq_dot, dq));
CoriGrav = simplify(jacobian(dL_dq_dot, [X_I; Eta]) * dq - dL_dq.' + dD_dq_dot.');

%% 状態方程式の生成と保存
dq_full = [dq; M \ (-CoriGrav + Q)];
dX      = simplify(dq_full);

all_vars = [x_I,  y_I,  z_I,  phi,  theta, psi, ...
            dx_I, dy_I, dz_I, dphi, dtheta, dpsi, ...
            F_x_I, F_y_I, F_z_I, tau_x_I, tau_y_I, tau_z_I, ...
            J_xx, J_yy, J_zz, m, g, mu_1, mu_2];

fdX = matlabFunction(dX, "Vars", all_vars);
save(fullfile(mat_dir, "dX.mat"), "fdX");
