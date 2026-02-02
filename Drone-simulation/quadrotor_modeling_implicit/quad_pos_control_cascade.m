%% quad_pos_control_cascade.m
% 位置制御（カスケード制御）の比較シミュレーション
% 姿勢制御は固定し、位置制御ゲインを変更して比較する

clear; close all;

%% 1. 前処理
% パス設定と関数ハンドル読み込み
mat_dir = fullfile(pwd, 'mat_data');
if ~exist(mat_dir, 'dir')
    % もしカレントディレクトリが違うなら、相対パスで探すなど対処
    error('mat_data directory not found based on pwd.');
end

load(fullfile(mat_dir, "M_matrix.mat"),  "fM");
load(fullfile(mat_dir, "CoriGrav.mat"),  "fCoriGrav");
load(fullfile(mat_dir, "R_BtoI.mat"),    "fR_BtoI");
load(fullfile(mat_dir, "Omega.mat"),     "fOmega");
load(fullfile(mat_dir, "P_BtoI.mat"),    "fP_BtoI");
load(fullfile(mat_dir, "P_ItoB.mat"),    "fP_ItoB");

%% 2. 定数設定
J_xx = 0.010;
J_yy = 0.015;
J_zz = 0.020;
m    = 0.7;
g    = 9.81;
const = struct('J_xx', J_xx, 'J_yy', J_yy, 'J_zz', J_zz, 'm', m, 'g', g);

%% 3. 初期条件
% 位置制御のステップ応答を見るため、初期位置0、目標位置を指定
X0      = [10; 10; 10];   % 初期位置を10m,10m,10mに設定
dX0     = [0; 0; 0];      % 初期速度は0
eta0    = [0; 0; 0];      % 初期角度も0
deta0   = [0; 0; 0];      % 初期角速度も0

q0   = [X0; eta0];
dq0  = [dX0; deta0];

%% 4. 制御パラメータ設定

% 固定する姿勢制御ゲイン (Case 2 from existing code)
kp_att_fixed = [6.0; 6.0; 6.0];
kd_att_fixed = [2.5; 2.5; 2.5];

% 目標位置 [x; y; z]
target_pos = [0.0; 0.0; 0.0];
target_yaw = 0;

% 比較用の位置制御ゲインセット
% Case 1: 収束が遅い (Overdamped, Very Slow)
% Kp=0.005, Kd=0.02.
kp_pos1 = [0.005; 0.005; 0.05];
kd_pos1 = [0.04; 0.04; 0.2];

% Case 2: 適切 (Stable and reasonably fast)
% Kp=0.04, Kd=0.2. Conservative gain to avoid divergence.
kp_pos2 = [0.04; 0.04; 1.0];
kd_pos2 = [0.2; 0.2; 2.0];

% Case 3: 発散または振動 (High Gain, Underdamped/Unstable)
kp_pos3 = [5.0; 5.0; 10.0];
kd_pos3 = [0.1; 0.1; 0.5];

% コントローラ構造体の作成
ctrl1 = create_controller(target_pos, target_yaw, kp_pos1, kd_pos1, kp_att_fixed, kd_att_fixed);
ctrl2 = create_controller(target_pos, target_yaw, kp_pos2, kd_pos2, kp_att_fixed, kd_att_fixed);
ctrl3 = create_controller(target_pos, target_yaw, kp_pos3, kd_pos3, kp_att_fixed, kd_att_fixed);

%% 5. シミュレーション設定
h      = 0.01;
t_end  = 60; % 収束を見るために時間を延長
gamma = 0.5;
beta  = 0.25;
max_iter = 10;
tol      = 1e-6;

%% 6. 実行

fprintf('カスケード位置制御シミュレーション開始...\n');

% Simulation 1
fprintf('Case 1: Low Gain...\n');
[t1, q1, dq1] = run_sim(q0, dq0, const, ctrl1, h, t_end, gamma, beta, max_iter, tol, fM, fCoriGrav, fR_BtoI, fP_BtoI, fP_ItoB);
fprintf('  -> Final Pos: [%.2f, %.2f, %.2f]\n', q1(end,1), q1(end,2), q1(end,3));

% Simulation 2
fprintf('Case 2: Good Gain...\n');
[t2, q2, dq2] = run_sim(q0, dq0, const, ctrl2, h, t_end, gamma, beta, max_iter, tol, fM, fCoriGrav, fR_BtoI, fP_BtoI, fP_ItoB);
fprintf('  -> Final Pos: [%.2f, %.2f, %.2f]\n', q2(end,1), q2(end,2), q2(end,3));

% Simulation 3
fprintf('Case 3: High Gain...\n');
[t3, q3, dq3] = run_sim(q0, dq0, const, ctrl3, h, t_end, gamma, beta, max_iter, tol, fM, fCoriGrav, fR_BtoI, fP_BtoI, fP_ItoB);
fprintf('  -> Final Pos: [%.2f, %.2f, %.2f]\n', q3(end,1), q3(end,2), q3(end,3));



%% 7. プロットと保存

fig1 = figure('Name', 'Position Control Comparison', 'Position', [100 100 1000 800]);

% X Position
subplot(3,1,1); hold on;
plot(t1, q1(:,1), 'b--', 'LineWidth', 1.5);
plot(t2, q2(:,1), 'g-', 'LineWidth', 2.0);
plot(t3, q3(:,1), 'r:', 'LineWidth', 1.5);
yline(target_pos(1), 'k--', 'Target');
title('X Position Response');
ylabel('X [m]');
legend('Low Gain', 'Good Gain', 'High Gain');
grid on;

% Y Position
subplot(3,1,2); hold on;
plot(t1, q1(:,2), 'b--', 'LineWidth', 1.5);
plot(t2, q2(:,2), 'g-', 'LineWidth', 2.0);
plot(t3, q3(:,2), 'r:', 'LineWidth', 1.5);
yline(target_pos(2), 'k--', 'Target');
title('Y Position Response');
ylabel('Y [m]');
grid on;

% Z Position
subplot(3,1,3); hold on;
plot(t1, q1(:,3), 'b--', 'LineWidth', 1.5);
plot(t2, q2(:,3), 'g-', 'LineWidth', 2.0);
plot(t3, q3(:,3), 'r:', 'LineWidth', 1.5);
yline(target_pos(3), 'k--', 'Target');
title('Z Position Response');
xlabel('Time [s]');
ylabel('Z [m]');
grid on;

% 姿勢の挙動も確認（Case 2のみなど）
fig2 = figure('Name', 'Attitude Response (Case 2)', 'Visible', 'off');
subplot(3,1,1); plot(t2, q2(:,4)*180/pi); title('Roll [deg]'); grid on;
subplot(3,1,2); plot(t2, q2(:,5)*180/pi); title('Pitch [deg]'); grid on;
subplot(3,1,3); plot(t2, q2(:,6)*180/pi); title('Yaw [deg]'); grid on;


% 保存（カレントディレクトリ）
saveas(fig1, 'position_control_result.png');
saveas(fig1, 'position_control_result.pdf');
saveas(fig2, 'attitude_control_result_case2.png');

% 保存（mat_dataディレクトリにも出力）
mat_data_dir = fullfile(pwd, 'mat_data');
if exist(mat_data_dir, 'dir')
    saveas(fig1, fullfile(mat_data_dir, 'position_control_result.png'));
    saveas(fig1, fullfile(mat_data_dir, 'position_control_result.pdf'));
end



%% ローカル関数定義

function ctrl = create_controller(pos, yaw, kp_p, kd_p, kp_a, kd_a)
    ctrl = struct();
    ctrl.target_pos = pos;
    ctrl.target_yaw = yaw;
    ctrl.kp_pos = kp_p;
    ctrl.kd_pos = kd_p;
    ctrl.kp_att = kp_a;
    ctrl.kd_att = kd_a;
end

function [t_out, q_out, dq_out] = run_sim(q0, dq0, const, ctrl, h, t_end, gamma, beta, max_iter, tol, fM, fC, fR, fPB, fPI)
    N = floor(t_end / h);
    q = q0;
    dq = dq0;
    
    % 初期加速度
    ddq = compute_accvar(q, dq, const, ctrl, fM, fC, fR, fPB, fPI);
    
    t_out = zeros(N+1, 1);
    q_out = zeros(N+1, 6);
    dq_out = zeros(N+1, 6);
    
    t_out(1) = 0;
    q_out(1,:) = q.';
    dq_out(1,:) = dq.';
    
    for n = 1:N
        % Newmark-beta integration step
        % Predictor
        ddq_new = ddq; 
        
        q_old = q;
        dq_old = dq;
        ddq_old = ddq;
        
        for iter = 1:max_iter
            % Update q, dq based on ddq_new prediction
            dq_new = dq_old + h * ((1-gamma)*ddq_old + gamma*ddq_new);
            q_new  = q_old + h*dq_old + h^2 * ((0.5-beta)*ddq_old + beta*ddq_new);
            
            % Compute forces and residual
            M = fM(q_new(1), q_new(2), q_new(3), q_new(4), q_new(5), q_new(6), ...
                   dq_new(1), dq_new(2), dq_new(3), dq_new(4), dq_new(5), dq_new(6), ...
                   const.J_xx, const.J_yy, const.J_zz, const.m, const.g);
                   
            CG = fC(q_new(1), q_new(2), q_new(3), q_new(4), q_new(5), q_new(6), ...
                    dq_new(1), dq_new(2), dq_new(3), dq_new(4), dq_new(5), dq_new(6), ...
                    const.J_xx, const.J_yy, const.J_zz, const.m, const.g);
            
            Q = compute_gen_force(q_new, dq_new, ctrl, const, fR, fPB, fPI);
            
            residual = M * ddq_new + CG - Q;
            
            if norm(residual) < tol
                break;
            end
            
            % Jacobian approx
            delta = -M \ residual;
            ddq_new = ddq_new + delta;
        end
        q = q_new;
        dq = dq_new;
        ddq = ddq_new;
        
        t_out(n+1) = n*h;
        q_out(n+1,:) = q.';
        dq_out(n+1,:) = dq.';
        
        % 発散チェック
        if any(abs(q(1:3)) > 100) || any(isnan(q))
            fprintf('  Diverged at t=%.2f\n', n*h);
            % 残りをNaN埋めして終了
            t_out(n+2:end) = [];
            q_out(n+2:end,:) = [];
            dq_out(n+2:end,:) = [];
            break;
        end
    end
end

function ddq = compute_accvar(q, dq, const, ctrl, fM, fC, fR, fPB, fPI)
    M = fM(q(1), q(2), q(3), q(4), q(5), q(6), ...
           dq(1), dq(2), dq(3), dq(4), dq(5), dq(6), ...
           const.J_xx, const.J_yy, const.J_zz, const.m, const.g);
    CG = fC(q(1), q(2), q(3), q(4), q(5), q(6), ...
            dq(1), dq(2), dq(3), dq(4), dq(5), dq(6), ...
            const.J_xx, const.J_yy, const.J_zz, const.m, const.g);
    Q = compute_gen_force(q, dq, ctrl, const, fR, fPB, fPI);
    ddq = M \ (Q - CG);
end

function Q = compute_gen_force(q, dq, ctrl, const, fR, fPB, fPI)
    X = q(1:3); eta = q(4:6);
    dX = dq(1:3); deta = dq(4:6);
    phi = eta(1); theta = eta(2); psi = eta(3);
    
    % --- Position Control ---
    e_p = ctrl.target_pos - X;
    e_v = -dX;
    
    acc_des = ctrl.kp_pos .* e_p + ctrl.kd_pos .* e_v;
    F_des_I = const.m * (acc_des + [0; 0; const.g]);
    
    % Thrust
    U1 = norm(F_des_I);
    if F_des_I(3) < 0, U1 = -U1; end
    
    % Attitude Target (Simplified)
    % phi_d approx F_y / -U1 (if psi=0)
    % theta_d approx F_x / U1 (if psi=0)
    % Including Yaw:
    % R * [0;0;U1] = [Fx; Fy; Fz]
    % Fx = U1 * (cos phi sin theta cos psi + sin phi sin psi)
    % Fy = U1 * (cos phi sin theta sin psi - sin phi cos psi)
    
    % Small angle approximation inversion
    % Fx/U1 = theta * cos_psi + phi * sin_psi
    % Fy/U1 = theta * sin_psi - phi * cos_psi
    % Apply rotation matrix [cos -sin; sin cos]^-1 = [cos sin; -sin cos]
    % theta = (Fx/U1)*cos + (Fy/U1)*sin
    % phi   = (Fx/U1)*sin - (Fy/U1)*cos  <-- check signs
    % Actually:
    % [Fx/U1; Fy/U1] = [cos_psi, sin_psi; sin_psi, -cos_psi] * [theta; phi] ? No.
    
    % Let's use simple geometric:
    % Forward force in Heading(psi) frame: F_fwd = Fx cos(psi) + Fy sin(psi)
    % Right force in Heading(psi) frame:   F_rgt = -Fx sin(psi) + Fy cos(psi)
    % F_fwd comes from Pitch down (negative theta? depends on axes). Typically F_fwd ~ U1 * theta if theta connects to X.
    % In standard aerospace R_BtoI (Z down):
    % R = [... ; ...; -sin(theta), sin(phi)cos(theta), cos(phi)cos(theta)]
    % Force in I = R * [0;0;-T] = [T sin(theta); -T sin(phi)cos(theta); -T cos(phi)cos(theta)]
    % => Fx = T theta, Fy = -T phi.
    
    % In this SIMULATION code:
    % We need to check coordinate system definition, but let's assume standard Quadrotor:
    % X forward, Y right/left, Z up/down?
    % g = 9.81.
    % If we look at existing `quad_simu3_compare_implicit.m`:
    % eta=[40deg, ...]. Initial pos=0.
    % It's hard to tell without seeing fR_BtoI.
    
    % Let's assume standard small angle relation derived from F_des:
    % Ux = F_des_I(1); Uy = F_des_I(2);
    % u_x = Ux / U1; u_y = Uy / U1;
    % theta_d = u_x * cos(psi) + u_y * sin(psi); 
    % phi_d   = u_x * sin(psi) - u_y * cos(psi);
    
    % Wait, phi creates force in Y direction (roll right -> force right or left?)
    % If Roll Right (phi > 0), force is usually to the right (if Z down) or left?
    % Let's assume standard right-hand rule.
    % phi > 0 -> Right wing down -> Push towards Right?
    % X forward, Y left, Z up?
    
    % Safe generic calculation:
    % F_des_local_x =  cos(psi)*F_des_I(1) + sin(psi)*F_des_I(2);
    % F_des_local_y = -sin(psi)*F_des_I(1) + cos(psi)*F_des_I(2);
    % theta_d = atan2(F_des_local_x, F_des_I(3)); % or small angle F/U
    % phi_d   = atan2(-F_des_local_y, sqrt(F_des_local_x^2 + F_des_I(3)^2)); % sign?
    
    % Let's use small angle with clamping.
    Fx_b =  cos(psi)*F_des_I(1) + sin(psi)*F_des_I(2);
    Fy_b = -sin(psi)*F_des_I(1) + cos(psi)*F_des_I(2);
    
    % Assuming Z is up (thrust opposes gravity), and Theta is pitch up (nose up).
    % If nose up (theta > 0), thrust creates BACKWARD force (-X). 
    % So to accelerate +X, we need theta < 0 (nose down).
    % Thus theta_d = - Fx_b / U1 (approx)
    
    % Roll: if phi > 0 (right wing down?), force to Right (-Y? or +Y?).
    % Standard: Y is left? If Z up, X fwd -> Y Left. 
    % If Y is Left, phi>0 (right down) pushes to Right (-Y).
    % So to accelerate +Y (Left), we need Left down (phi < 0).
    % Thus phi_d = - Fy_b / U1
    
    % Use saturation
    max_ang = 30 * pi/180;
    theta_d = -Fx_b / const.g / const.m; % Normalize properly? No, U1 is the force.
    phi_d   =  Fy_b / const.g / const.m; % Sign guess.
    
    % Correction: Force F = U1 * sin(theta) ~ m g theta.
    % So theta = F / (mg).
    % Sign: if theta moves X (+), theta = +F/mg. If X is forward and Theta is Nose Up,
    % Nose Up -> Backward Force. We want Forward Force. So Nose Down (Theta < 0).
    % So theta_d = - Fx_b / (m*g).
    
    % Let's stick with:
    % 符号の再考:
    % 座標系が Z-up, Y-left, theta-pitch-down(>0) の場合: theta > 0 -> Fx > 0. (theta_d = +Fx)
    % 座標系が Z-up, Y-left, theta-pitch-up(>0)   の場合: theta > 0 -> Fx < 0. (theta_d = -Fx)
    % 前回 -Fx で発散したので、今回は +Fx を試す。
    theta_d =  Fx_b / U1;
    phi_d   = -Fy_b / U1; % Y軸も同様に符号反転の可能性ありだが、とりあえずそのまま(-Fy)か(+Fy)か。前回も-Fyだった。
    % phi: roll. If Y-left. phi>0 (right-down). Force to Right(-Y).
    % To go +Y (Left), need phi<0 (Left-down). 
    % Fy > 0 (Force Left). phi_d need to be < 0. 
    % phi_d = -Fy. This matches previous code.

    
    theta_d = max(min(theta_d, max_ang), -max_ang);
    phi_d   = max(min(phi_d,   max_ang), -max_ang);
    
    eta_d = [phi_d; theta_d; ctrl.target_yaw];
    
    % --- Attitude Control ---
    e_att = eta - eta_d;
    d_att = deta;
    
    u_torq = -ctrl.kp_att .* e_att - ctrl.kd_att .* d_att;
    
    % Conversions for simulation model
    % Original: u = ...; input_sim = u * 100/(472); torq = complex_scaling * input_sim
    % To keep fixed gains working as intended, we should feed `u_torq` as the `u` in original code.
    % But original code `u` output was NOT physical torque? 
    % "motor_torq = ... * input_sim; ... Tau_b = motor_torq"
    % So `u` was proportional to torque.
    % We will assume our `u_torq` computed with fixed gains is the same scale as `u` in original.
    % (Kp=[6,6,6] is likely for the `u` scale, not physical Nm)
    
    u_val = u_torq;
    
    % Calculate physical torque using the model in original code
    input_sim = u_val * 100/(1023-551);
    motor_torq = 4 * diag([0.1425, 0.090, 1]) * ...
                 diag([0.02219, 0.02219, 0.0003533]) * input_sim;
                 
    F_b = [0; 0; U1];
    Tau_b = motor_torq;
    % 慣性座標系への変換
    R_BtoI = fR(phi, theta, psi);
    P_BtoI = fPB(phi, theta, psi);
    F_I = R_BtoI * F_b;
    Tau_I = P_BtoI * Tau_b;
    Q = [F_I; Tau_I];
end
