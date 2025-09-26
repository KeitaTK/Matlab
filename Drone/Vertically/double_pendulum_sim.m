function double_pendulum_sim
   % パラメータ設定
   g=9.81;         % 重力加速度　[m/s^2]
   L1=1.0;         % 上の棒の長さ　[m]
   L2=1.0;         % 下の棒の長さ  [m]
   m1=1.0;         % 上の質量　　[kg]
   m2=1.0;         % 下の質量　　[kg]

   % 初期条件: [theta1,omega1,theta2,omega2]
   y0=[pi/2,0,pi/2,0];

   % 時間設定　
   tspan=[0,20];

   % 微分方程式を解く
   [T, Y] = ode45(@(t, y) equations(t, y, m1, m2, L1, L2, g), tspan, y0);
    

    % アニメーション描画
    for i = 1:5:length(T)
        clf;
        x1 = L1 * sin(Y(i,1));
        y1 = -L1 * cos(Y(i,1));
        x2 = x1 + L2 * sin(Y(i,3));
        y2 = y1 - L2 * cos(Y(i,3));
        
        plot([0 x1 x2], [0 y1 y2], '-o', 'LineWidth', 2);
        axis equal;
        axis([-2 2 -2.5 0.5]);
        title(sprintf('Time: %.2f sec', T(i)));
        drawnow;
    end
end

% 微分方程式（運動方程式）
function dydt = equations(~, y, m1, m2, L1, L2, g)
    theta1 = y(1);
    omega1 = y(2);
    theta2 = y(3);
    omega2 = y(4);
    
    delta = theta2 - theta1;

    den1 = (m1 + m2) * L1 - m2 * L1 * cos(delta)^2;
    den2 = (L2 / L1) * den1;

    domega1 = (m2 * L1 * omega1^2 * sin(delta) * cos(delta) + ...
               m2 * g * sin(theta2) * cos(delta) + ...
               m2 * L2 * omega2^2 * sin(delta) - ...
               (m1 + m2) * g * sin(theta1)) / den1;

    domega2 = (-m2 * L2 * omega2^2 * sin(delta) * cos(delta) + ...
               (m1 + m2) * (g * sin(theta1) * cos(delta) - ...
               L1 * omega1^2 * sin(delta) - ...
               g * sin(theta2))) / den2;

    dydt = [omega1; domega1; omega2; domega2];
end