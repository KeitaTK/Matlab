%% 
% quad_lag_7_implicit.m
% ラグランジュの運動方程式によるクアッドコプター（ドローン）の運動モデル
% 陰解法実装用：空気抵抗なし（mu_1, mu_2を削除）、慣性モーメント3軸異なる値
% 慣性座標系での運動方程式を導出し、matファイルとして保存
% 
% 変更点：
% - 空気抵抗項（mu_1, mu_2）を削除
% - 慣性モーメントは J_xx, J_yy, J_zz の3軸で異なる値を使用

clear;
% 位置・速度・角度・慣性・力・トルクのシンボリック変数定義
syms x_I y_I z_I real;
syms dx_I dy_I dz_I real;
syms phi theta psi real;
syms dphi dtheta dpsi real;
syms m g real;
syms J_xx J_yy J_zz real;
syms F_x_I F_y_I F_z_I real;
syms tau_x_I tau_y_I tau_z_I real;
syms t real;

%% 一般化座標と速度
X_I   = [x_I; y_I; z_I];
Eta   = [phi; theta; psi];
dX_I  = [dx_I; dy_I; dz_I];
dEta  = [dphi; dtheta; dpsi];
dq    = [dX_I; dEta];
Q     = [F_x_I; F_y_I; F_z_I; tau_x_I; tau_y_I; tau_z_I];
J     = diag([J_xx, J_yy, J_zz]);  % 3軸異なる慣性モーメント

%% 角速度ベクトルの読み込みと生成
mat_dir = fullfile(pwd, 'mat_data');
load(fullfile(mat_dir, "Omega.mat"), "fOmega");
Omega = fOmega(phi, theta, psi, dphi, dtheta, dpsi);

%% エネルギー項（空気抵抗なし）
W_trans = 0.5 * m * (dX_I.' * dX_I);           % 並進運動エネルギー
W_rot   = 0.5 * Omega.' * J * Omega;           % 回転運動エネルギー
W       = W_trans + W_rot;                      % 総運動エネルギー
U       = m * g * z_I;                          % ポテンシャルエネルギー

%% ラグランジアン（空気抵抗項なし）
L = simplify(W - U);

%% オイラー–ラグランジュ方程式
dL_dq_dot = jacobian(L, dq);
dL_dq     = jacobian(L, [X_I; Eta]);

% 空気抵抗項がないため、散逸関数 D は 0
M        = expand(jacobian(dL_dq_dot, dq));    % 質量行列
CoriGrav = simplify(jacobian(dL_dq_dot, [X_I; Eta]) * dq - dL_dq.');  % コリオリ・重力項

%% 質量行列と慣性行列の保存（陰解法で使用）
fM = matlabFunction(M, "Vars", [x_I, y_I, z_I, phi, theta, psi, ...
                                dx_I, dy_I, dz_I, dphi, dtheta, dpsi, ...
                                J_xx, J_yy, J_zz, m, g]);
save(fullfile(mat_dir, "M_matrix.mat"), "fM");

%% コリオリ・重力項の保存（陰解法で使用）
fCoriGrav = matlabFunction(CoriGrav, "Vars", [x_I, y_I, z_I, phi, theta, psi, ...
                                               dx_I, dy_I, dz_I, dphi, dtheta, dpsi, ...
                                               J_xx, J_yy, J_zz, m, g]);
save(fullfile(mat_dir, "CoriGrav.mat"), "fCoriGrav");

%% 状態方程式の生成と保存（比較用・検証用）
dq_full = [dq; M \ (-CoriGrav + Q)];
dX      = simplify(dq_full);

all_vars = [x_I,  y_I,  z_I,  phi,  theta, psi, ...
            dx_I, dy_I, dz_I, dphi, dtheta, dpsi, ...
            F_x_I, F_y_I, F_z_I, tau_x_I, tau_y_I, tau_z_I, ...
            J_xx, J_yy, J_zz, m, g];

fdX = matlabFunction(dX, "Vars", all_vars);
save(fullfile(mat_dir, "dX.mat"), "fdX");

disp('quad_lag_7_implicit.m: 運動方程式の導出完了');
disp('  - 空気抵抗なし（mu_1, mu_2 削除）');
disp('  - 慣性モーメント: J_xx, J_yy, J_zz（3軸異なる値）');
disp('  - 質量行列 M, コリオリ・重力項 CoriGrav を保存');
