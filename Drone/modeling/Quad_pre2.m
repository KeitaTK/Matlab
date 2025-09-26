%% 
% このコードはドローンの運動方程式を求める過程で必要な、慣性座標系から機体座標系からの変換と、角速度ベクトルの形を作り、ファイルにアップデートする。

clear;
% 位置
syms x_b y_b z_b real;                  % _bは機体座標系
syms phi theta psi real;                         % 一般座標系から見た機体の傾き(機体座標系と初期値の慣性座標系は一致しているものとする)
syms dphi dtheta dpsi real;
syms t real;
%% 
% 慣性座標系から機体座標系に変換する回転行列を作る。
% 
% X_I = R_BtoI * X_b で変換できる。

Rx = [1, 0, 0;
      0, cos(phi), -sin(phi);
      0, sin(phi), cos(phi)];
Ry = [cos(theta), 0, sin(theta);
      0, 1, 0;
      -sin(theta), 0, cos(theta)];
Rz = [cos(psi), -sin(psi), 0;
      sin(psi), cos(psi), 0;
      0, 0, 1];
R_BtoI = Rz*Ry*Rx;
% R_BtoI = (Rx*Ry*Rz);
% R_BtoI
% R_ItoB = simplify(inv(R_BtoI));
% simplify(R_BtoI*R_ItoB);  % 検算
%% 
% 機体の姿勢を、慣性座標系から機体座標系に変換

X_b = [x_b; y_b; z_b];

X_I = R_BtoI * X_b;
%% 
% 機体の角速度ベクトル ω を求める。

syms a b c d e f real;
t1 = [a; b; c];
t2 = [d e f];
t3 = cross(t1, t2);    % t1 × t2
Scew = jacobian(t3, t2);
VecToScew = @(x) [0, -x(3), x(2); ...
               x(3), 0, -x(1); ...
              -x(2), x(1), 0];
ScewToVec = @(X) [X(3,2); X(1,3); X(2,1)];

eta = [phi; theta; psi];
deta = [dphi; dtheta; dpsi];
dR = matDiff(R_BtoI, eta, deta);
Omega = ScewToVec(R_BtoI'*dR);
%% 
% 機体座標系でのトルクを、固定座標系でのトルクに変換する行列を作る

P_ItoB = jacobian(Omega, deta);
P_BtoI = simplify(inv(P_ItoB));
%% 
% 作った回転行列を保存。

vars1 = {eta}; % 変数の順番を phi theta psi  に指定
fR_BtoI = matlabFunction(R_BtoI,"Vars",vars1); % 関数ハンドルを作成
save("R_BtoI.mat", "fR_BtoI"); % 関数ハンドルを保存
%% 
% 作ったωを保存。

vars2 = {[eta;deta]}; % 変数の順番を phi,theta,psi,dphi,dtheta,dpsi  に指定
fOmega = matlabFunction(Omega,"Vars",vars2);
save("Omega.mat", "fOmega");
%% 
% 作ったP_BtoIを保存。

fP_BtoI = matlabFunction(P_BtoI,"Vars",vars1);
save("P_BtoI.mat", "fP_BtoI");

fP_ItoB = matlabFunction(P_ItoB,"Vars",vars1); % 関数ハンドルを作成
save("P_ItoB.mat", "fP_ItoB"); % 関数ハンドルを保存
%% 
% 

function dM = matDiff(M, x, dx)
    sM = size(M); %row, col
    dM = sym(zeros(sM));
    for ii=1:sM(2)  % 列毎
        dM(:,ii) = simplify(jacobian(M(:,ii), x))*dx;
        
    end
end