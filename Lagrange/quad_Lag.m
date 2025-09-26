clear;
% 位置
syms x_b(t) y_b(t) z_b(t)                % _bは機体座標系
syms x_I(t) y_I(t) z_I(t)                % _Iは慣性座標系
syms phi_I(t) theta_I(t) psi_I(t)        % 一般座標系から見た機体の傾き
syms m g 
syms J_xx J_yy J_zz
syms phi theta psi real;
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
R_BtoI

X_b = [x_b; y_b; z_b];
X_I = [x_I; y_I; z_I];

Theta_b = [phi_I; theta_I; psi_I];

J = [J_xx 0 0;
     0 J_yy 0;
     0 0 J_zz ];

V_b = diff(X_b,t);

Omega_b = diff(Theta_b,t);

R_v = [cos(psi_I)]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

W_1 = 0.5*m*V_b*(V_b');
W_2 = 0.5*(Omega_b')*J*Omega_b;

U = m*g*X




