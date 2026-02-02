%% quad_simu3_implicit.m
% クアッドコプターの剛体運動シミュレーション（陰解法実装）
% 平均加速度法（ニューマークβ法: γ=1/2, β=1/4）を使用
% 
% 特徴：
% - 無条件安定な陰解法（ニューマークβ法）
% - 収束計算（ニュートン・ラフソン法）により高精度
% - 慣性モーメント3軸異なる値、空気抵抗なし
%
% ニューマークβ法パラメータ：
%   γ = 1/2  : 平均加速度法
%   β = 1/4  : 無条件安定、中立安定（エネルギー散逸なし）

clear; close all;

%% 1. 前処理（関数ハンドルの読み込み）
mat_dir = fullfile(pwd, 'mat_data');
load(fullfile(mat_dir, "M_matrix.mat"),  "fM");
load(fullfile(mat_dir, "CoriGrav.mat"),  "fCoriGrav");
load(fullfile(mat_dir, "R_BtoI.mat"),    "fR_BtoI");
load(fullfile(mat_dir, "Omega.mat"),     "fOmega");
load(fullfile(mat_dir, "P_BtoI.mat"),    "fP_BtoI");
load(fullfile(mat_dir, "P_ItoB.mat"),    "fP_ItoB");

%% 2. 定数設定（3軸異なる慣性モーメント）
J_xx = 0.010;   % X軸慣性モーメント [kg·m²]
J_yy = 0.015;   % Y軸慣性モーメント [kg·m²]
J_zz = 0.020;   % Z軸慣性モーメント [kg·m²]
m    = 0.7;     % 質量 [kg]
g    = 9.81;    % 重力加速度 [m/s²]

const = struct('J_xx', J_xx, 'J_yy', J_yy, 'J_zz', J_zz, 'm', m, 'g', g);

%% 3. 初期条件
X0      = [0; 0; 0];                    % 位置 [m]
dX0     = [0; 0; 0];                    % 速度 [m/s]
eta0    = [40; 20; 10] * pi/180;        % ロール・ピッチ・ヨー [rad]
deta0   = [0; 0; 0];                    % 角速度 (Euler angle rates) [rad/s]

% 初期加速度を計算
q0   = [X0; eta0];
dq0  = [dX0; deta0];
ddq0 = compute_acceleration(q0, dq0, const, fM, fCoriGrav, fR_BtoI, fP_BtoI, fP_ItoB);

%% 4. 制御器設定
etad = [0; 0; 0];      % 目標角度 [rad]
kp   = [5; 16; 50];    % 比例ゲイン
kd   = [6; 6; 5];      % 微分ゲイン
controller = struct('etad', etad, 'kp', kp, 'kd', kd);

%% 5. シミュレーション設定
h      = 0.01;        % 時間刻み [s]
t_end  = 10;          % 終了時刻 [s]
N      = t_end / h;   % ステップ数

% ニューマークβ法パラメータ
gamma = 0.5;          % 平均加速度法
beta  = 0.25;         % 無条件安定

% 収束判定
max_iter = 10;        % 最大反復回数
tol      = 1e-6;      % 収束許容誤差

%% 6. 結果保存用配列
t_array    = zeros(N+1, 1);
q_array    = zeros(N+1, 6);
dq_array   = zeros(N+1, 6);
ddq_array  = zeros(N+1, 6);

% 初期値を保存
t_array(1)   = 0;
q_array(1,:)   = [X0; eta0].';
dq_array(1,:)  = [dX0; deta0].';
ddq_array(1,:) = ddq0.';

%% 7. 陰解法による時間積分（ニューマークβ法）
fprintf('陰解法シミュレーション開始...\n');
fprintf('  方法: 平均加速度法（ニューマークβ法, γ=%.2f, β=%.2f）\n', gamma, beta);
fprintf('  時間刻み: %.4f [s], 終了時刻: %.1f [s]\n', h, t_end);

for n = 1:N
    t_n = (n-1) * h;
    t_np1 = n * h;
    
    % 現在の状態
    q_n   = q_array(n, :).';
    dq_n  = dq_array(n, :).';
    ddq_n = ddq_array(n, :).';
    
    % 予測ステップ（ニューマークβ法の初期推定）
    ddq_np1 = ddq_n;  % 初期推定：前ステップと同じ加速度
    
    % ニュートン・ラフソン法による収束計算
    for iter = 1:max_iter
        % ニューマークβ法による速度・位置の計算
        dq_np1 = dq_n + h * ((1 - gamma) * ddq_n + gamma * ddq_np1);
        q_np1  = q_n + h * dq_n + h^2 * ((0.5 - beta) * ddq_n + beta * ddq_np1);
        
        % 外力・トルクの計算
        Q_np1 = compute_generalized_force(q_np1, dq_np1, controller, const, ...
                                          fR_BtoI, fP_BtoI, fP_ItoB);
        
        % 残差の計算（つり合い式）
        M_np1   = compute_mass_matrix(q_np1, dq_np1, const, fM);
        CG_np1  = compute_coriolis_gravity(q_np1, dq_np1, const, fCoriGrav);
        
        residual = M_np1 * ddq_np1 + CG_np1 - Q_np1;
        
        % 収束判定
        if norm(residual) < tol
            break;
        end
        
        % ヤコビ行列（ニューマークβ法の係数を含む有効質量行列）
        % J = M + γ*h*C + β*h²*K ≈ M（簡略化：C, Kの寄与を無視）
        J_eff = M_np1;
        
        % 修正量の計算
        delta_ddq = -J_eff \ residual;
        
        % 加速度の更新
        ddq_np1 = ddq_np1 + delta_ddq;
    end
    
    % 収束しない場合の警告
    if iter == max_iter && norm(residual) >= tol
        warning('時刻 t=%.4f で収束しませんでした (残差=%.2e)', t_np1, norm(residual));
    end
    
    % 結果を保存
    t_array(n+1)     = t_np1;
    q_array(n+1, :)    = q_np1.';
    dq_array(n+1, :)   = dq_np1.';
    ddq_array(n+1, :)  = ddq_np1.';
end

fprintf('シミュレーション完了\n\n');

%% 8. 結果のプロット
% 角度
figure(1);
subplot(3,1,1), plot(t_array, q_array(:,4)*180/pi), grid on;
title('ロール角 φ'), xlabel('時刻 [s]'), ylabel('角度 [deg]');
subplot(3,1,2), plot(t_array, q_array(:,5)*180/pi), grid on;
title('ピッチ角 θ'), xlabel('時刻 [s]'), ylabel('角度 [deg]');
subplot(3,1,3), plot(t_array, q_array(:,6)*180/pi), grid on;
title('ヨー角 ψ'), xlabel('時刻 [s]'), ylabel('角度 [deg]');

% 角速度
figure(2);
subplot(3,1,1), plot(t_array, dq_array(:,4)*180/pi), grid on;
title('角速度 dφ/dt'), xlabel('時刻 [s]'), ylabel('角速度 [deg/s]');
subplot(3,1,2), plot(t_array, dq_array(:,5)*180/pi), grid on;
title('角速度 dθ/dt'), xlabel('時刻 [s]'), ylabel('角速度 [deg/s]');
subplot(3,1,3), plot(t_array, dq_array(:,6)*180/pi), grid on;
title('角速度 dψ/dt'), xlabel('時刻 [s]'), ylabel('角速度 [deg/s]');

% 位置
figure(3);
subplot(3,1,1), plot(t_array, q_array(:,1)), grid on;
title('位置 x'), xlabel('時刻 [s]'), ylabel('位置 [m]');
subplot(3,1,2), plot(t_array, q_array(:,2)), grid on;
title('位置 y'), xlabel('時刻 [s]'), ylabel('位置 [m]');
subplot(3,1,3), plot(t_array, q_array(:,3)), grid on;
title('位置 z'), xlabel('時刻 [s]'), ylabel('位置 [m]');

% 速度
figure(4);
subplot(3,1,1), plot(t_array, dq_array(:,1)), grid on;
title('速度 dx/dt'), xlabel('時刻 [s]'), ylabel('速度 [m/s]');
subplot(3,1,2), plot(t_array, dq_array(:,2)), grid on;
title('速度 dy/dt'), xlabel('時刻 [s]'), ylabel('速度 [m/s]');
subplot(3,1,3), plot(t_array, dq_array(:,3)), grid on;
title('速度 dz/dt'), xlabel('時刻 [s]'), ylabel('速度 [m/s]');

%% ========== 補助関数 ==========

%% 初期加速度の計算
function ddq = compute_acceleration(q, dq, const, fM, fCoriGrav, fR_BtoI, fP_BtoI, fP_ItoB)
    X   = q(1:3);
    eta = q(4:6);
    dX   = dq(1:3);
    deta = dq(4:6);
    
    % 質量行列
    M = compute_mass_matrix(q, dq, const, fM);
    
    % コリオリ・重力項
    CG = compute_coriolis_gravity(q, dq, const, fCoriGrav);
    
    % 初期状態での外力（ホバリング状態を仮定）
    controller_dummy = struct('etad', [0;0;0], 'kp', [5;16;50], 'kd', [6;6;5]);
    Q = compute_generalized_force(q, dq, controller_dummy, const, ...
                                  fR_BtoI, fP_BtoI, fP_ItoB);
    
    % 加速度の計算
    ddq = M \ (-CG + Q);
end

%% 質量行列の計算
function M = compute_mass_matrix(q, dq, const, fM)
    X   = q(1:3);
    eta = q(4:6);
    dX   = dq(1:3);
    deta = dq(4:6);
    
    M = fM(X(1), X(2), X(3), eta(1), eta(2), eta(3), ...
           dX(1), dX(2), dX(3), deta(1), deta(2), deta(3), ...
           const.J_xx, const.J_yy, const.J_zz, const.m, const.g);
end

%% コリオリ・重力項の計算
function CG = compute_coriolis_gravity(q, dq, const, fCoriGrav)
    X   = q(1:3);
    eta = q(4:6);
    dX   = dq(1:3);
    deta = dq(4:6);
    
    CG = fCoriGrav(X(1), X(2), X(3), eta(1), eta(2), eta(3), ...
                   dX(1), dX(2), dX(3), deta(1), deta(2), deta(3), ...
                   const.J_xx, const.J_yy, const.J_zz, const.m, const.g);
end

%% 一般化力の計算（制御入力）
function Q = compute_generalized_force(q, dq, controller, const, ...
                                       fR_BtoI, fP_BtoI, fP_ItoB)
    X   = q(1:3);
    eta = q(4:6);
    dX   = dq(1:3);
    deta = dq(4:6);
    
    % 座標変換行列
    R_BtoI = fR_BtoI(eta(1), eta(2), eta(3));
    P_BtoI = fP_BtoI(eta(1), eta(2), eta(3));
    P_ItoB = fP_ItoB(eta(1), eta(2), eta(3));
    
    % 機体座標系での角度と角速度
    eta_b  = P_ItoB * eta;
    deta_b = P_ItoB * deta;
    
    % PD制御
    u = -controller.kp .* (eta_b - controller.etad) - controller.kd .* deta_b;
    
    % モーター入力への変換（元のコードと同じスケーリング）
    input_sim = u * 100/(1023-551);
    
    % モーター力・トルク
    motor_torq = 4 * diag([0.1425, 0.090, 1]) * ...
                 diag([0.02219, 0.02219, 0.0003533]) * input_sim;
    F_b   = [0; 0; 1] * 80 * 0.02219 * 4;  % 推力
    Tau_b = motor_torq;                     % トルク
    
    % 慣性座標系への変換
    F_I   = R_BtoI * F_b;
    Tau_I = P_BtoI * Tau_b;
    
    Q = [F_I; Tau_I];
end
