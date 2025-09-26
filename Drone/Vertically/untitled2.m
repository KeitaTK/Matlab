% vertical_pid_sim.m
% 1-DOF mass under gravity, PID height control
clear; clc;
%% 1. Parameters ----------------------------------------------------------
m      = 1.0;          % mass [kg]
g      = 9.81;         % gravity [m/s^2]
Kp     = 30;           % PID gains
Ki     = 5;
Kd     = 8;
Ts     = 0.001;        % sampling time [s]
T_end  = 5.0;          % simulation duration [s]
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
   % Force saturation (thrust can only push upward)
   u(k) = max(u_unsat,0);
   % System dynamics (Euler integration)
   a = (u(k) / m) - g;       % acceleration
   v(k+1) = v(k) + a*Ts;     % velocity update
   y(k+1) = y(k) + v(k)*Ts;  % position update
end
e(N) = y_ref - y(N);   % last error value
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

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline"}
%---
