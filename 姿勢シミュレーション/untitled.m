clear;
tic;
T = readtable('output45.csv'); % テーブルを読み込む
% 配列に変換
time_val = table2array(T(:,1));
euler_val = table2array(T(:, [2 3 4]));
motor_val = table2array(T(:, [5 6 7 8]));

% パラメータの定義
tspan = [time_val(1), time_val(end)]; % 時間範囲をデータから取得
% 初期条件 (Eta = [Phi, Theta, Psi], dEta = [dPhi, dTheta, dPsi])
Eta0 = [pi*20/180; pi*30/180; -pi*10/180];
dEta0 = [0; 0; 0];
x0 = [Eta0; dEta0]; % nは状態変数から削除

% 微分方程式の解を求める
[t1, xx] = ode45(@(t,x) func1(t, x, motor_val, time_val), tspan, x0); % motor_valとtime_valを渡す

figure(1); clf; hold on;
subplot(3,1,1);
plot(t1, xx(:,1));
title('ロール角 (φ)');
xlabel('時間 (秒)');
ylabel('角度 (rad)');

subplot(3,1,2);
plot(t1, xx(:,2));
title('ピッチ角 (θ)');
xlabel('時間 (秒)');
ylabel('角度 (rad)');

subplot(3,1,3);
plot(t1, xx(:,3));
title('ヨー角 (Ψ)');
xlabel('時間 (秒)');
ylabel('角度 (rad)');

figure(2); clf; hold on;
subplot(3,1,1);
plot(t1, xx(:,4));
title('dPhi (dPhi)');
xlabel('時間 (秒)');
ylabel('角速度 (rad/s)');

subplot(3,1,2);
plot(t1, xx(:,5));
title('dTheta (dTheta)');
xlabel('時間 (秒)');
ylabel('角速度 (rad/s)');

subplot(3,1,3);
plot(t1, xx(:,6));
title('dPsi (dPsi)');
xlabel('時間 (秒)');
ylabel('角速度 (rad/s)');

toc;

% func1関数の定義
function dx = func1(t, x, motor_val, time_val)
    % 線形補間を使ってmotor_valを取得
    for i = 1:length(time_val)-1
      if t >= time_val(i) && t <= time_val(i+1)
        n = i;
        break;
      end
    end
    if t < time_val(1)
        n = 1;
    elseif t > time_val(end)
        n = length(time_val)-1;
    end

    % x = [Phi, Theta, Psi, dPhi, dTheta, dPsi]
    Eta = x(1:3);    % Eta = [Phi, Theta, Psi]
    dEta = x(4:6);   % dEta = [dPhi, dTheta, dPsi]
    Phi = Eta(1);    Theta = Eta(2);    Psi = Eta(3);
    dPhi = dEta(1);  dTheta = dEta(2);  dPsi = dEta(3);
    g = 9.81; % 重力加速度 (m/s^2)
    m = 1.7; % 機体重量 (kg)
    Ib = diag([0.2, 0.5, 0.3]); % 慣性モーメント (kg*m^2)
    P = [Ib(1,1),0,-1/2*Ib(1,1)*sin(Theta);
        0,Ib(2,2)*cos(Phi)+1/2*Ib(2,2)*sin(Phi)*cos(Theta),0;
        0,0,-Ib(1,1)*sin(Theta)+1/2*Ib(2,2)*sin(Phi)*cos(Theta)-1/2*Ib(3,3)*cos(Phi)*cos(Theta)]; % 慣性モーメントの逆行列
    r = 0.1685*(ones(1, 4)); % モータ軸の距離 (m)
    Jr = 1/2 * m * r.^2; % ジャイロ効果の慣性モーメント (スカラー)
    Omegar = [100; 100; 100; 100]; % ロータの角速度

    h1 =  -1/2*Ib(1,1)*dTheta*dPsi*cos(Theta)+Ib(2,2)*sin(Phi)*(1/2*dTheta*cos(Phi)+1/2*dPsi*sin(Phi)*cos(Theta))-Ib(2,2)*cos(Phi)*(1/2*dPsi*cos(Phi)*cos(Theta)-1/2*dTheta*sin(Phi)) -Ib(3,3)*cos(Phi)*(1/2*dTheta*sin(Theta)-1/2*dPsi*cos(Phi)*cos(Theta))-Ib(3,3)*sin(Phi)*(1/2*dTheta*cos(Phi)+1/2*dPsi*sin(Phi)*cos(Theta));
    h2 =  Ib(2,2)*(-dPhi*dTheta*sin(Phi)+1/2*dPhi*dTheta*cos(Phi)*cos(Theta)+sin(Theta)*(1/2*dTheta*cos(Phi)+1/2*dPsi*cos(Theta)*sin(Phi))-cos(Theta)*sin(Phi)*(1/2*dPsi*cos(Theta)*sin(Phi)+1/2*dTheta*cos(Phi)))+Ib(3,3)*(1/2*dPhi*dTheta*cos(Phi)+sin(Theta)*cos(Phi)*(1/2*dTheta*sin(Phi)-1/2*dPsi*cos(Phi)*cos(Theta)));
    h3 =   Ib(1,1)*(-dTheta*dPsi*cos(Theta)+dPsi*sin(Theta))-Ib(2,2)*cos(Theta)*sin(Phi)*(1/2*dTheta*cos(Phi)+1/2*dPsi*cos(Theta)*sin(Phi))+Ib(3,3)*(1/2*dTheta*dPsi*sin(Theta)*cos(Phi)+cos(Theta)*cos(Phi)*(1/2*dTheta*sin(Phi)-1/2*dPsi*cos(Phi)*cos(Theta)));
    h = [ h1;h2;h3];

    etad=[0;0;pi*0/180];  %目標角
    kp = [5; 16; 50]; % 角度制御の比例ゲイン
    kd = [6; 6; 5]; % 角度制御の微分ゲイン

    input_motor = -kp .* (Eta-etad) - kd .* dEta; % 551~1023の間での実機の入力にプラスマイナスする値
    input_sim = input_motor*100/(1023-551);  % 0~100の間でのシミュレーションの入力
    motor_torq = 4*diag([0.1425,0.090,-1])*(diag([2.219170753809196,2.219170753809196,3.533461241187303e-04]) * input_sim);  % 重心からの距離(m) * モーターの生み出す力(N)

    Tau = motor_torq;
    Tau(1) = motor_val(n,2); %外部入力に変更

    % d2x (加速度成分)
    d2x = P\ (Tau - h);
    % dx = [dEta; d2x] -> [dPhi, dTheta, dPsi, d2Phi, d2Theta, d2Psi]
    dx = [dEta; d2x];
end