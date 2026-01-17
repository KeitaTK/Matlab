% vertical_pid_sim_with_animation.m
% 1-DOF mass under gravity, PID height control + animation
clear; clc;

%% 1. Parameters ----------------------------------------------------------
m      = 1.0;          % mass [kg]
g      = 9.81;         % gravity [m/s^2]
Kp     = 30;           % PID gains
Ki     = 30;
Kd     = 10;
Ts     = 0.01;        % sampling time [s]
T_end  = 100;         % simulation duration [s] ←延長
y_ref  = 2.0;          % target height [m]
y0     = 0.0;          % initial height [m]
v0     = 0.0;          % initial velocity [m/s]

%% 2. Pre-allocation ------------------------------------------------------
t  = 0:Ts:T_end;
N  = numel(t);
y  = zeros(1,N);  y(1) = y0;
v  = zeros(1,N);  v(1) = v0;
u  = zeros(1,N);
e  = zeros(1,N);
int_e = 0;              % integral term
prev_e = 0;             % for derivative

%% 3. Main loop -----------------------------------------------------------
for k = 1:N-1
   % Error
   e(k) = y_ref - y(k);
   % Integral & derivative
   int_e   = int_e + e(k)*Ts;
   der_e   = (e(k) - prev_e)/Ts;
   prev_e  = e(k);
   % PID control law
   u_unsat = Kp*e(k) + Ki*int_e + Kd*der_e;
   u(k) = max(u_unsat,0);  % saturation
   % System dynamics (Euler integration)
   a = (u(k) / m) - g;
   v(k+1) = v(k) + a*Ts;
   y(k+1) = y(k) + v(k)*Ts;
end
e(N) = y_ref - y(N);

%% 4. Plotting ------------------------------------------------------------
figure;
subplot(3,1,1);
plot(t,y,'b', t, y_ref*ones(size(t)),'--r');
ylabel('Height y [m]');
legend('y','y_{ref}','Location','best'); grid on;
subplot(3,1,2);
plot(t,u,'m');
ylabel('Thrust u [N]'); grid on;
subplot(3,1,3);
plot(t,e,'k');
xlabel('Time [s]'); ylabel('Error e [m]'); grid on;
sgtitle('Vertical Position Control with PID');

% %% 5. Animation -----------------------------------------------------------
% figure;
% % for k = 1:round(0.01/Ts):N
% for k = 1:round(0.05/Ts):N
% 
% 
%     clf;
%     % ドローンを長方形で描画
%     rectangle('Position',[-0.1, y(k)-0.05, 0.2, 0.1], ...
%               'Curvature', 0.2, 'FaceColor', [0.2 0.6 1]);
%     hold on;
%     % 地面と目標線
%     plot([-0.5 0.5],[0 0],'k','LineWidth',2); % ground
%     plot([-0.5 0.5],[y_ref y_ref],'--r','LineWidth',1.5); % target
%     % 表示設定
%     axis equal;
%     axis([-0.5 0.5 -0.2 3]);
%     title(sprintf('Time: %.2f s', t(k)));
%     ylabel('Height [m]');
%     drawnow;
% end

