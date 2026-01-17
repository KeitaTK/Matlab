%% drone_slung_load_simulation.m
% ドローン＋吊り荷システムのLQR制御とシミュレーション

function drone_slung_load_simulation()
    clear; close all; clc;

    %% 1. パラメータ設定
    m_d = 1.0;      % ドローン質量 [kg]
    m_l = 0.5;      % 吊り荷質量 [kg]
    L   = 1.0;      % ひも長さ [m]
    g   = 9.81;     % 重力加速度 [m/s^2]
    M   = m_d + m_l;

    % 線形化定数
    alpha = m_l * g / M;
    gamma = g / L;

    %% 2. 線形系の状態空間モデル (10次元)
    % 状態ベクトル x = [xd; yd; zd; dxd; dyd; dzd; theta; phi; dtheta; dphi]
    A = zeros(10);

    % 1) 位置 → 速度
    A(1:3,4:6) = eye(3);

    % 2) 速度 → 吊り荷角度（θ, φ）結合（x, y方向）
    A(4:5,7:8) = -alpha * eye(2);

    % 3) 角度 → 角速度
    A(7:8,9:10) = eye(2);

    % 4) 角速度 → 角度復元力（振り子項）
    A(9,7)  = -gamma;   % d²θ/dθ
    A(10,8) = -gamma;   % d²φ/dφ

    % 入力行列 B
    B = zeros(10,3);
    B(4:6,1:3)  = eye(3) / M;       % 並進入力 → ドローン加速度
    B(9:10,1:2) = -eye(2) / (L * M);% 並進入力 → 吊り荷角加速度

    %% 3. LQRゲイン計算
    Q = diag([10,10,10, 1,1,1, 50,50, 5,5]);  % 角度重視
    R = diag([1,1,1]);
    [K,~,~] = lqr(A, B, Q, R);

    %% 4. 参照軌道設定（ホバリング）
    x_ref = @(t) zeros(10,1);

    %% 5. シミュレーション設定
    tspan = [0 20];       % 0–20秒
    x0 = zeros(10,1);
    x0(7) = 0.2;          % θ初期角 [rad]
    x0(8) = -0.1;         % φ初期角 [rad]
    ode_opts = odeset('RelTol',1e-6,'AbsTol',1e-6);

    %% 6. 非線形動力学＋LQR制御付きODE実行
    [t, X] = ode45(@dynamics, tspan, x0, ode_opts);

    %% 7. 結果プロット
    % 吊り荷角度
    figure; hold on; grid on;
    plot(t, X(:,7)*180/pi, 'r', 'LineWidth',1.5);
    plot(t, X(:,8)*180/pi, 'b', 'LineWidth',1.5);
    xlabel('時間 [s]'); ylabel('吊り荷角度 [deg]');
    legend('\theta','\phi');
    title('吊り荷角度の時間応答');

    % ドローン軌道
    figure; hold on; grid on;
    plot3(X(:,1), X(:,2), X(:,3), 'k', 'LineWidth',1.2);
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    title('ドローン軌道'); view(3);

    %% 8. 3Dアニメーション
    figure; axis equal; grid on; hold on;
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('ドローン＋吊り荷 3Dアニメーション');
    for i = 1:10:length(t)
        cla;
        % ドローン
        plot3(X(i,1), X(i,2), X(i,3), 'ro','MarkerSize',8,'MarkerFaceColor','r');
        % 吊り荷
        load_pos = X(i,1:3)' + L*[sin(X(i,7)); sin(X(i,8)); -cos(X(i,7))*cos(X(i,8))];
        plot3(load_pos(1), load_pos(2), load_pos(3), 'bo','MarkerSize',6,'MarkerFaceColor','b');
        % ひも
        plot3([X(i,1) load_pos(1)], [X(i,2) load_pos(2)], [X(i,3) load_pos(3)], 'k-');
        drawnow;
    end

    %% ネスト関数：非線形動力学
    function dx = dynamics(t, x)
        % 状態分解
        dxd   = x(4:6);
        theta = x(7);
        phi   = x(8);
        dth   = x(9);
        dph   = x(10);

        % 制御入力
        e = x - x_ref(t);
        u = -K * e;    % [Fx; Fy; Fz]

        % ドローン加速度（並進＋重力＋吊り荷遠心力）
        ddxd = u / m_d + [0;0;-g] + ...
               [ -(m_l/m_d)*L*dth^2*sin(theta);
                 -(m_l/m_d)*L*dph^2*sin(phi);
                  (m_l/m_d)*L*(dth^2*cos(theta)+dph^2*cos(phi)) ];

        % 吊り荷角加速度
        ddth = -(g/L)*theta + u(1)/(L*m_d);
        ddph = -(g/L)*phi   + u(2)/(L*m_d);

        dx = [dxd; ddxd; dth; dph; ddth; ddph];
    end
end
