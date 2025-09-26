syms phi theta psi f real;

% 回転行列を作成
Rx = [1, 0, 0;
      0, cos(phi), -sin(phi);
      0, sin(phi), cos(phi)];
Ry = [cos(theta), 0, sin(theta);
      0, 1, 0;
      -sin(theta), 0, cos(theta)];
Rz = [cos(psi), -sin(psi), 0;
      sin(psi), cos(psi), 0;
      0, 0, 1];
R_BtoI = Rz*Ry*Rx; % 回転行列
R_z = R_BtoI(3,3);
R_xy = R_BtoI(1:2,3);
fR_BtoI = matlabFunction(R_BtoI,'Vars',{[phi; theta; psi]}); % 回転行列の値を計算できる関数
save("functions.mat", "fR_BtoI");

% VecToScew = @(x) [0, -x(3), x(2); ...
%                x(3), 0, -x(1); ...
%               -x(2), x(1), 0];
% ScewToVec = @(X) [X(3,2); X(1,3); X(2,1)];
% 
% syms dphi dtheta dpsi real;
% eta = [phi; theta; psi];
% deta = [dphi; dtheta; dpsi];
% dR = matDiff(R_BtoI, eta, deta);
% 
% omega = ScewToVec(R_BtoI'*dR);
% P_ItoB = simplify(jacobian(omega, deta));
% 
% P_BtoI = simplify(inv(P_ItoB));
