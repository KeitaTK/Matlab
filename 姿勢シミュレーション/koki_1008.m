clear;
tic;
% パラメータの定義
tspan = [0 5]; % 時間範囲
% 初期条件 (Eta = [Phi, Theta, Psi], dEta = [dPhi, dTheta, dPsi])
Eta0 = [pi*20/180; pi*30/180; -pi*90/180];
dEta0 = [0; 0; 0];
x0 = [Eta0; dEta0];
% 微分方程式の解を求める
[t1, xx] = ode45(@func1, tspan, x0);
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

% func1関数の定義
function dx = func1(~, x)
     % x = [Phi, Theta, Psi, dPhi, dTheta, dPsi]
    Eta = x(1:3);    % Eta = [Phi, Theta, Psi]
    dEta = x(4:6);   % dEta = [dPhi, dTheta, dPsi]
    Phi = Eta(1);    Theta = Eta(2);    Psi = Eta(3);
    dPhi = dEta(1);  dTheta = dEta(2);  dPsi = dEta(3);
    g = 9.81; % 重力加速度 (m/s^2)
    m = 1; % ボールの質量 (kg)
    Ib = [0.2, 0, 0; 0, 0.5, 0; 0, 0, 0.3]; % 慣性モーメント (kg*m^2)
    P = [Ib(1,1),0,-1/2*Ib(1,1)*sin(Theta);
        0,Ib(2,2)*cos(Phi)+1/2*Ib(2,2)*sin(Phi)*cos(Theta),0;
        0,0,-Ib(1,1)*sin(Theta)+1/2*Ib(2,2)*sin(Phi)*cos(Theta)-1/2*Ib(3,3)*cos(Phi)*cos(Theta)]; % 慣性モーメントの逆行列
    r = [0.08, 0.08, 0.08, 0.08]; % モータ軸の距離 (m)
    Jr = 1/2 * m * r.^2; % ジャイロ効果の慣性モーメント (スカラー)
    Omegar = [100; 100; 100; 100]; % ロータの角速度

    % x = [Phi, Theta, Psi, dPhi, dTheta, dPsi]
    % Eta = x(1:3);    % Eta = [Phi, Theta, Psi]
    % dEta = x(4:6);   % dEta = [dPhi, dTheta, dPsi]
    % Phi = Eta(1);    Theta = Eta(2);    Psi = Eta(3);
    % dPhi = dEta(1);  dTheta = dEta(2);  dPsi = dEta(3);
    % 力学項 h

        h1 =  -1/2*Ib(1,1)*dTheta*dPsi*cos(Theta)+Ib(2,2)*sin(Phi)*(1/2*dTheta*cos(Phi)+1/2*dPsi*sin(Phi)*cos(Theta))-Ib(2,2)*cos(Phi)*(1/2*dPsi*cos(Phi)*cos(Theta)-1/2*dTheta*sin(Phi)) -Ib(3,3)*cos(Phi)*(1/2*dTheta*sin(Theta)-1/2*dPsi*cos(Phi)*cos(Theta))-Ib(3,3)*sin(Phi)*(1/2*dTheta*cos(Phi)+1/2*dPsi*sin(Phi)*cos(Theta));%+ Jr * Omegar * dTheta;

        h2 =  Ib(2,2)*(-dPhi*dTheta*sin(Phi)+1/2*dPhi*dTheta*cos(Phi)*cos(Theta)+sin(Theta)*(1/2*dTheta*cos(Phi)+1/2*dPsi*cos(Theta)*sin(Phi))-cos(Theta)*sin(Phi)*(1/2*dPsi*cos(Theta)*sin(Phi)+1/2*dTheta*cos(Phi)))+Ib(3,3)*(1/2*dPhi*dTheta*cos(Phi)+sin(Theta)*cos(Phi)*(1/2*dTheta*sin(Phi)-1/2*dPsi*cos(Phi)*cos(Theta)));% - Jr * Omegar * dPhi  ;

        h3 =   Ib(1,1)*(-dTheta*dPsi*cos(Theta)+dPsi*sin(Theta))-Ib(2,2)*cos(Theta)*sin(Phi)*(1/2*dTheta*cos(Phi)+1/2*dPsi*cos(Theta)*sin(Phi))+Ib(3,3)*(1/2*dTheta*dPsi*sin(Theta)*cos(Phi)+cos(Theta)*cos(Phi)*(1/2*dTheta*sin(Phi)-1/2*dPsi*cos(Phi)*cos(Theta)));


        h = [ h1;h2;h3];
    %       -Ib(2,2)*sin(Phi)*(1/2*dTheta*cos(Phi)+1/2*dPsi*sin(Phi)*cos(Theta))+Ib(2,2)*cos(Phi)*(1/2*dPsi*cos(Phi)*cos(Theta)-1/2*dTheta*sin(Phi)) ...
    %       +Ib(3,3)*cos(Phi)*(1/2*dTheta*sin(Theta)-1/2*dPsi*cos(Phi)*cos(Theta))+Ib(3,3)*sin(Phi)*(1/2*dTheta*cos(Phi)+1/2*dPsi*sin(Phi)*cos(Theta)) ...
    %       + Jr * Omegar * dTheta ;
    %        Ib(2,2)*(-dPhi*dTheta*sin(Phi)+1/2*dPhi*dTheta*cos(Phi)*cos(Theta)-sin(Theta)*(1/2*dTheta*cos(Phi)+1/2*dPsi*cos(Theta)*sin(Phi))+cos(Theta)*sin(Phi)*(1/2*dPsi*cos(Theta)*sin(Phi)+1/2*dTheta*cos(Phi))) ...
    %       +Ib(3,3)*(1/2*dPhi*dTheta*cos(Phi)-sin(Theta)*cos(Phi)*(1/2*dTheta*sin(Phi)-1/2*dPsi*cos(Phi)*cos(Theta))) ...
    %       - Jr * Omegar * dPhi ; 
    %        Ib(1,1)*(-dTheta*dPsi*cos(Theta)-dPsi*sin(Theta)) ...
    %       -Ib(2,2)*cos(Theta)*sin(Phi)*(1/2*dTheta*cos(Phi)+1/2*dPsi*cos(Theta)*sin(Phi)) ...
    %       +Ib(3,3)*(1/2*dTheta*dPsi*sin(Theta)*cos(Phi)-cos(Theta)*cos(Phi)*(1/2*dTheta*sin(Phi)-1/2*dPsi*cos(Phi)*cos(Theta)))];
   

    etad=[0;0;pi*0/180];  %目標角
    kp = [5; 16; 15]; % 角度制御の比例ゲイン
    kd = [6; 6; 5]; % 角度制御の微分ゲイン

    input_motor = -kp .* (Eta-etad) - kd .* dEta;
    % disp(input_motor)



    
    % トルク Tau (EtaとdEtaに依存)
    Tau = P * (-kp .* (Eta-etad) - kd .* dEta)+h
 

    % d2x (加速度成分)
    d2x = P\ (Tau - h);
    % dx = [dEta; d2x] -> [dPhi, dTheta, dPsi, d2Phi, d2Theta, d2Psi]
    dx = [dEta; d2x];
end
 
