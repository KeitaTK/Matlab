clear
load("functions.mat");

disp(fR_BtoI);



tspan=0:0.01:10;


eta0 = [pi/6;pi/6;pi/6];
deta0 = [0; 0; 0];
Att0 = [eta0; deta0];

X0=[6;3;-1];
dX0=[0;0;0];
Pos0=[X0;dX0];

Current0=[Att0;Pos0];

% [ta, Current] = ode45(@func1, tspan, Current0, R_z, R_xy);
[ta, Current] = ode45(@(t,Current) func1(t, Current, fR_BtoI), tspan, Current0);


figure(1); clf; hold on;
subplot(3,1,1);
plot(ta, Current(:,1));
title('ロール角 (φ)');
xlabel('時間 (秒)');
ylabel('角度 (rad)');
 
subplot(3,1,2);
plot(ta, Current(:,2));
title('ピッチ角 (θ)');
xlabel('時間 (秒)');
ylabel('角度 (rad)');
 
subplot(3,1,3);
plot(ta, Current(:,3));
title('ヨー角 (Ψ)');
xlabel('時間 (秒)');
ylabel('角度 (rad)');

figure(2); clf; hold on;

subplot(3,1,1);
plot(ta, Current(:,4));
title('dPhi (dPhi)');
xlabel('時間 (秒)');
ylabel('角速度 (rad/s)');

subplot(3,1,2);
plot(ta, Current(:,5));
title('dTheta (dTheta)');
xlabel('時間 (秒)');
ylabel('角速度 (rad/s)');

subplot(3,1,3);
plot(ta, Current(:,6));
title('dPsi (dPsi)');
xlabel('時間 (秒)');
ylabel('角速度 (rad/s)');

figure(3); clf; hold on;
subplot(3,1,1);
plot(ta, Current(:,7));
title('位置(x)');
xlabel('時間 (秒)');
ylabel(' 位置(m)');
 
subplot(3,1,2);
plot(ta, Current(:,8));
title('位置(y)');
xlabel('時間 (秒)');
ylabel(' 位置(m)');
 
subplot(3,1,3);
plot(ta, Current(:,9));
title('位置(z)');
xlabel('時間 (秒)');
ylabel(' 位置(m)');

figure(4); clf; hold on;

subplot(3,1,1);
plot(ta, Current(:,10));
title('速度 (dx)');
xlabel('時間 (秒)');
ylabel('速度 (m/s)');

subplot(3,1,2);
plot(ta, Current(:,11));
title('速度 (dy)');
xlabel('時間 (秒)');
ylabel('速度 (m/s)');

subplot(3,1,3);
plot(ta, Current(:,12));
title('速度 (dz)');
xlabel('時間 (秒)');
ylabel('速度 (m/s)');




function dCurrent = func1(t, Current, fR_BtoI)
    X_target=[0;0;0]; % 目標位置

    eta = Current(1:3);    % Eta = [Phi, Theta, Psi]
    deta = Current(4:6);   % dEta = [dPhi, dTheta, dPsi]
    X = Current(7:9);
    dX = Current(10:12);
    

    phi = Current(1);    theta = Current(2);    psi = Current(3);
    dphi = Current(4);  dtheta = Current(5);  dpsi = Current(6);
    x = Current(7); y = Current(8); z = Current(9);
    dx = Current(10); dy = Current(11); dz = Current(12);

    X_diff = [x;y;z] - X_target;
    


    kp_p=diag([1, 1, 1]); % kp_x,kp_y,kp_z
    kd_p=diag([1, 1, 1]); % kd_x,kd_y,kd_z
    
    ddX_d=-kp_p*X_diff-kd_p*[dx;dy;dz];
    
    m=2;
    g=9.81;
    
   
    

    u_x_d=ddX_d(1);
    u_y_d=ddX_d(2);
    u_z_d=ddX_d(3);
    R_psi=[cos(psi),-sin(psi);sin(psi),cos(psi)];
    
    

    
    
    
    u_xy_d=[u_x_d;u_y_d];
    B=R_psi\(u_xy_d/(u_z_d-g));
    theta_d=atan(B(1));
    phi_d=-atan(cos(theta_d)*B(2));
    % theta_d=0;
    % phi_d=0;
    psi_d=0;
    eta_d=[phi_d;theta_d;psi_d];
    
    R = fR_BtoI([phi; theta; psi]);
    R_z = R(3,3);
    f=m*(u_z_d-g)/R_z;
    R_xy = R(1:2,3);
    u_xy=R_xy*f/m;
    u_z=R_z*f/m+g;
    
    ddX=[u_xy;u_z];
    
    
    Ib = [0.2, 0, 0; 0, 0.5, 0; 0, 0, 0.3]; % 慣性モーメント (kg*m^2)
    P = [Ib(1,1),0,-1/2*Ib(1,1)*sin(theta);
         0,Ib(2,2)*cos(phi)+1/2*Ib(2,2)*sin(phi)*cos(theta),0;
         0,0,-Ib(1,1)*sin(theta)+1/2*Ib(2,2)*sin(phi)*cos(theta)-1/2*Ib(3,3)*cos(phi)*cos(theta)];
            
   
        kp_a = 10*[8; 8; 8]; % 位置制御の比例ゲイン
        kd_a = 10*[2; 2; 2]; % 速度制御の微分ゲイン
    
        h1 =  -1/2*Ib(1,1)*dtheta*dpsi*cos(theta)+Ib(2,2)*sin(phi)*(1/2*dtheta*cos(phi)+1/2*dpsi*sin(phi)*cos(theta))-Ib(2,2)*cos(phi)*(1/2*dpsi*cos(phi)*cos(theta)-1/2*dtheta*sin(phi)) -Ib(3,3)*cos(phi)*(1/2*dtheta*sin(theta)-1/2*dpsi*cos(phi)*cos(theta))-Ib(3,3)*sin(phi)*(1/2*dtheta*cos(phi)+1/2*dpsi*sin(phi)*cos(theta));

        h2 =  Ib(2,2)*(-dphi*dtheta*sin(phi)+1/2*dphi*dtheta*cos(phi)*cos(theta)+sin(theta)*(1/2*dtheta*cos(phi)+1/2*dpsi*cos(theta)*sin(phi))-cos(theta)*sin(phi)*(1/2*dpsi*cos(theta)*sin(phi)+1/2*dtheta*cos(phi)))+Ib(3,3)*(1/2*dphi*dtheta*cos(phi)+sin(theta)*cos(phi)*(1/2*dtheta*sin(phi)-1/2*dpsi*cos(phi)*cos(theta)));

        h3 =   Ib(1,1)*(-dtheta*dpsi*cos(theta)+dpsi*sin(theta))-Ib(2,2)*cos(theta)*sin(phi)*(1/2*dtheta*cos(phi)+1/2*dpsi*cos(theta)*sin(phi))+Ib(3,3)*(1/2*dtheta*dpsi*sin(theta)*cos(phi)+cos(theta)*cos(phi)*(1/2*dtheta*sin(phi)-1/2*dpsi*cos(phi)*cos(theta)));


        h = [ h1;h2;h3];
        
    
   
    P_BtoI=[1,sin(phi)*sin(theta)/cos(theta),cos(phi)*sin(theta)/cos(theta);
            0,cos(phi),-sin(phi);
            0,sin(phi)/cos(theta),cos(phi)/cos(theta)];


    Tau = P_BtoI\(P * (-kp_a .* (eta-eta_d) - kd_a .* deta)+h);
    d2eta = P\ (P_BtoI*Tau - h);
    
    

    
    dCurrent = [deta;d2eta;dX;ddX];
    % disp(dCurrent);



    
end

function dM = matDiff(M, x, dx)
    sM = size(M); %row, col
    dM = sym(zeros(sM));
    for ii=1:sM(2)
        dM(:,ii) = simplify(jacobian(M(:,ii), x))*dx;
    end
end