clear;
load('Current1.mat');
load('Current2.mat');



figure(11); clf; % figure(1)を作成またはクリア

% ロール角 (φ)
subplot(3, 1, 1); % 3行1列の1番目のsubplot
plot(ta1, Current1(:,4)*180/pi(), 'b-', 'LineWidth', 1.5); % 実験値 (青実線)
hold on;
plot(ta2, Current2(:,4)*180/pi(), 'r--', 'LineWidth', 1.5); % シミュレーション値 (赤破線)
hold off;
% title('(a) I_xx=I_yy=I_zz=0.001','FontSize',14);
xlabel('Time [s]','FontSize',13);
ylabel('Roll Angle [degree]','FontSize',12);
% legend('Experimental Data', 'Simulation Data','FontSize',10); % 凡例も英語表記に

grid on;

% ピッチ角 (θ)
subplot(3, 1, 2); % 3行1列の2番目のsubplot
plot(ta1, Current1(:,5)*180/pi(), 'b-', 'LineWidth', 1.5); % 実験値 (青実線)
hold on;
plot(ta2, Current2(:,5)*180/pi(), 'r--', 'LineWidth', 1.5); % シミュレーション値 (赤破線)
hold off;
% title('ピッチ角 (θ)');
xlabel('Time [s]','FontSize',13);
ylabel('Pitch Angle [degree]','FontSize',12);
legend('Kp=[10,32,30],Kd = [12,12,10]', ['Kp=[20,32,70],' ...
    'Kd = [12,12,60]'],'FontSize',13, 'Location', 'SouthEast');
grid on;

% ヨー角 (ψ)
subplot(3, 1, 3); % 3行1列の3番目のsubplot
plot(ta1, Current1(:,6)*180/pi(), 'b-', 'LineWidth', 1.5); % 実験値 (青実線)
hold on;
plot(ta2, Current2(:,6)*180/pi(), 'r--', 'LineWidth', 1.5); % シミュレーション値 (赤破線)
hold off;
% title('ヨー角 (ψ)');
xlabel('Time [s]','FontSize',13);
ylabel(' Yaw Angle [degree]','FontSize',12);
% legend('Experimental Data', 'Simulation Data','FontSize',10); % 凡例も英語表記に

grid on;