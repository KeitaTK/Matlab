clear;

% fdxをロード
load("dX.mat", "fdX");

% 初期値
eta0 = [pi/6; pi/6; pi/6];
deta0 = [0; 0; 0];
Att0 = [eta0; deta0];
X0 = [6; 3; -1];
dX0 = [0; 0; 0];
Pos0 = [X0; dX0];
Current0 = [Pos0; Att0];

% シミュレーション時間
tspan = 0:0.01:10;

% 定数 (create_fdX.m で定義したものと同じ値を使用)
J_xx = 1; J_yy = 2; J_zz = 3; % 例
m = 1; g = 9.81; mu_1 = 0.1; mu_2 = 0.01; % 例
const = [J_xx; J_yy; J_zz; m; g; mu_1; mu_2];

% 外力 (例: すべて0)
F_b = [0; 0; 0];
Tau_b = [0; 0; 0];
Q = [F_b; Tau_b];

% 回転行列と角速度を計算する関数ハンドルをロード
load("R_BtoI.mat", "fR_BtoI");
load("Omega.mat", "fOmega");

% ode45でシミュレーション
options = odeset('RelTol',1e-6,'AbsTol',1e-6);
[ta, Current] = ode45(@(t, Current) system_dynamics(t, Current, Q, const, fR_BtoI, fOmega, fdX), tspan, Current0,options);

% シミュレーション結果の取り出し
X_b_sim = Current(:, 1:3);
Eta_sim = Current(:, 4:6);
dX_b_sim = Current(:, 7:9);
dEta_sim = Current(:, 10:12);

% 結果のプロット (例)
figure;
subplot(3,1,1); plot(ta, Eta_sim(:,1)); title('phi');
subplot(3,1,2); plot(ta, Eta_sim(:,2)); title('theta');
subplot(3,1,3); plot(ta, Eta_sim(:,3)); title('psi');

% system_dynamics 関数の定義 (重要)
function dCurrent = system_dynamics(t, Current, Q, const, fR_BtoI, fOmega, fdx)
    X_b = Current(1:3);
    Eta = Current(4:6);
    dX_b = Current(7:9);
    dEta = Current(10:12);
    X = [X_b; Eta; dX_b; dEta];

    R_BtoI = fR_BtoI(Eta);
    Omega = fOmega([Eta; dEta]);

    vars = {[X; Q; const]};
    fdx_val = fdx(vars{:});
    dCurrent = fdx_val;
end