% Quad_pre2.m
% ロール・ピッチ・ヨー角から回転行列R_BtoIを計算し、座標変換・角速度ベクトル・トルク変換行列を導出
% R_BtoI, Omega, P_BtoI, P_ItoBの関数ハンドルをmatファイルとして保存
% 
% 詳細はREADME.md参照

clear;
% 位置と角度のシンボリック変数定義
syms x_b y_b z_b real;
syms phi theta psi real;
syms dphi dtheta dpsi real;
syms a b c d e f real;
syms t real;

%% 回転行列の定義
Rx = [1, 0, 0;
      0, cos(phi), -sin(phi);
      0, sin(phi),  cos(phi)];
Ry = [cos(theta), 0, sin(theta);
      0,          1, 0;
     -sin(theta), 0, cos(theta)];
Rz = [cos(psi), -sin(psi), 0;
      sin(psi),  cos(psi), 0;
      0,              0,   1];
R_BtoI = Rz * Ry * Rx;  % 慣性座標系への変換

%% 位置変換
X_b = [x_b; y_b; z_b];
X_I = R_BtoI * X_b;

%% 角速度ベクトル ω の導出
t1 = [a; b; c];
t2 = [d; e; f];
t3 = cross(t1, t2);

VecToScew = @(x) [  0,   -x(3),  x(2);
                  x(3),    0,   -x(1);
                 -x(2), x(1),     0];
ScewToVec = @(X) [X(3,2); X(1,3); X(2,1)];

eta  = [phi; theta; psi];
deta = [dphi; dtheta; dpsi];
dR   = matDiff(R_BtoI, eta, deta);
Omega = ScewToVec(R_BtoI' * dR);

%% トルク変換行列
P_ItoB = jacobian(Omega, deta);
P_BtoI = simplify(inv(P_ItoB));

%% 関数ハンドルの生成と保存
vars1 = [phi, theta, psi];
vars2 = [phi, theta, psi, dphi, dtheta, dpsi];

fR_BtoI = matlabFunction(R_BtoI, "Vars", vars1);
fOmega   = matlabFunction(Omega,  "Vars", vars2);
fP_BtoI  = matlabFunction(P_BtoI,  "Vars", vars1);
fP_ItoB  = matlabFunction(P_ItoB,  "Vars", vars1);

% 保存用フォルダを作成
mat_dir = fullfile(pwd, 'mat_data');
if ~exist(mat_dir, 'dir')
    mkdir(mat_dir);
end

save(fullfile(mat_dir, "R_BtoI.mat"),  "fR_BtoI");
save(fullfile(mat_dir, "Omega.mat"),   "fOmega");
save(fullfile(mat_dir, "P_BtoI.mat"),  "fP_BtoI");
save(fullfile(mat_dir, "P_ItoB.mat"),  "fP_ItoB");

%% 補助関数：行列の変数微分
function dM = matDiff(M, x, dx)
    sM = size(M);
    dM = sym(zeros(sM));
    for ii = 1:sM(2)
        dM(:,ii) = simplify(jacobian(M(:,ii), x) * dx);
    end
end
