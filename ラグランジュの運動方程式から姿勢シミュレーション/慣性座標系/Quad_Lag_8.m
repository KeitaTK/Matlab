% �������l�������^�������������߂�
% D = 0 �ɂ���Ƒ����Ȃ��ɂȂ�
% �������W�n�ōl����

clear;

% �V���{���b�N�ϐ��̒�`
syms x_I y_I z_I real                      % �ʒu�i�������W�n�j
syms dx_I dy_I dz_I real                   % ���x�i�������W�n�j
syms phi theta psi real                    % �I�C���[�p
syms dphi dtheta dpsi real                 % �p���x����
syms m g real                              % ���ʁA�d�͉����x
syms J_xx J_yy J_zz real                   % �������[�����g
syms F_x_I F_y_I F_z_I real                % ���i�́i�������W�n�j
syms tau_x_I tau_y_I tau_z_I real          % ���[�����g�i�������W�n�j
syms mu_1 mu_2 real                        % �����W��
syms t real                                % ����

% ��ʉ����W�ƈ�ʉ��͂��`
X_I = [x_I; y_I; z_I];
Eta = [phi; theta; psi];
q = [X_I; Eta];                           % ��ʉ����W

dX_I = [dx_I; dy_I; dz_I];
dEta = [dphi; dtheta; dpsi];
dq = [dX_I; dEta];                        % ��ʉ����x

F_I = [F_x_I; F_y_I; F_z_I];
Tau_I = [tau_x_I; tau_y_I; tau_z_I];
Q = [F_I; Tau_I];

J = [J_xx 0 0;
     0 J_yy 0;
     0 0 J_zz];

% �@�̂̊p���x�x�N�g�� �� �����߂�
load("Omega.mat", "fOmega");              % �֐��n���h�������[�h
Omega = fOmega([Eta; dEta]);

% �^���G�l���M�[�����߂�B���i�Ɖ�]�̍��v
W_1 = 0.5 * m * (dX_I)' * dX_I;
W_2 = 0.5 * (Omega') * J * Omega;
W = W_1 + W_2;

% �ʒu�G�l���M�[�����߂�
U = m * g * z_I;                          % �ȗ����Fz_I�����̂�

% �����G�l���M�[�����߂�BD = 0�ɂ���ƁA�����Ȃ�
D = (0.5 * mu_1 * (dX_I') * dX_I) + (0.5 * mu_2 * (Omega') * Omega);
% D = 0;  % �����Ȃ��̏ꍇ�͂�������R�����g�A�E�g

% ���O���W�A�����v�Z
L = W - U;
L = simplify(L);

% �I�C���[�E���O�����W���������̍��ӂ��v�Z
% M �͊����s��CCoriGrav �͉��S�͂ƃR���I���́A�|�e���V��������󂯂�̘͂a�ɑ������𑫂�������
dL_dq_dot = jacobian(L, dq);
dL_dq = jacobian(L, q);
dD_dq_dot = jacobian(D, dq);

M = jacobian(dL_dq_dot, dq);
M = expand(M);

h = simplify(jacobian(dL_dq_dot, q) * dq);
g_term = simplify(dL_dq.');

CoriGrav = jacobian(dL_dq_dot, q) * dq - dL_dq.' + dD_dq_dot.';
CoriGrav = simplify(CoriGrav);

% ������֐���ۑ�
X = [X_I; Eta; dX_I; dEta];
dX = [dq; M \ (-CoriGrav + Q)];
dX = simplify(dX);

% �֐��n���h���쐬���̕ϐ����X�g�i�C���Łj
% �S�ẴV���{���b�N�ϐ����ʂɎw��
vars_list = [x_I, y_I, z_I, phi, theta, psi, ...
             dx_I, dy_I, dz_I, dphi, dtheta, dpsi, ...
             F_x_I, F_y_I, F_z_I, tau_x_I, tau_y_I, tau_z_I, ...
             J_xx, J_yy, J_zz, m, g, mu_1, mu_2];

% �֐��n���h���쐬
fdX = matlabFunction(dX, 'Vars', {vars_list});
save("dX.mat", "fdX");

fprintf('�^���������̊֐��n���h���𐳏�ɍ쐬�E�ۑ����܂����B\n');
