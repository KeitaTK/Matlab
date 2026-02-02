%% export_eq_latex.m
% 運動方程式の数式をLaTeX形式で出力するスクリプト

clear;
mat_dir = fullfile(pwd, 'mat_data');
load(fullfile(mat_dir, "Omega.mat"), "fOmega");

syms phi theta psi dphi dtheta dpsi real
syms J_xx J_yy J_zz m g real
syms x_I y_I z_I dx_I dy_I dz_I real

% 1. 角速度ベクトル
Omega = fOmega(phi, theta, psi, dphi, dtheta, dpsi);
latex_Omega = latex(simplify(Omega));

% 2. 質量行列 M (回転部分のみ抽出)
% 並進部分は自明 (m*I) なので省略し、回転部分 J_rot を計算する
syms J_xx J_yy J_zz real
J_body = diag([J_xx, J_yy, J_zz]);
W_rot = 0.5 * Omega.' * J_body * Omega;

% 一般化速度（回転）
dEta = [dphi; dtheta; dpsi];
dL_rot_dEta_dot = jacobian(W_rot, dEta);
M_rot = jacobian(dL_rot_dEta_dot, dEta);
M_rot_simplified = simplify(M_rot);

latex_M_rot = latex(M_rot_simplified);

% 3. コリオリ項・重力項 (の一部)
% ポテンシャル U = m*g*z_I
% 重力項 G(q) = dU/dq = [0;0;mg; 0;0;0] (並進zのみ)
% コリオリ力は複雑なので形式のみ言及するか、特定の成分だけ出す

% 出力
fid = fopen('eq_latex.txt', 'w');
fprintf(fid, '%% Omega\n%s\n\n', latex_Omega);
fprintf(fid, '%% M_rot\n%s\n\n', latex_M_rot);
fclose(fid);

disp('LaTeX export done.');
type eq_latex.txt
