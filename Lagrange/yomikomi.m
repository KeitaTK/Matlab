clear;

format long

load('qqq1.mat');


x1=qqq(:,1)*5-5;
y1=-qqq(:,4)/4;
p1 = polyfit(x1, y1, 1);
y1=y1-p1(2);
p1 = polyfit(x1, y1, 1);
x_fit1 = linspace(min(x1), max(x1),100);
y_fit1 = polyval(p1, x_fit1);
figure(1);
plot(x1,y1,x_fit1,y_fit1);
% title('プロペラ1枚推力');
equation1 = sprintf('y1 = %.10fx',p1(1));
% text(0,2,equation1,'Fontsize',12);
y1 = p1(1) * 50 + p1(2); % 数値計算
disp(p1(1));

legend('Measured Thrust','Linear fit','Location','northwest','FontSize',12); % 凡例を追加、位置とフォントサイズを指定

% 軸ラベルの追加
xlabel('Input Values','FontSize',12); % X軸ラベル
ylabel('Thrust [N])','FontSize',12);    % Y軸ラベル

grid on;

y2=qqq(:,7)/2;

p2 = polyfit(x1, y2, 1);
y2 = y2 - p2(2);
p2 = polyfit(x1, y2, 1);
x_fit2 = linspace(min(x1), max(x1),100);
y_fit2 = polyval(p2, x_fit2);
figure(2);
plot(x1,y2,x_fit2,y_fit2);
% title('ヨー方向のトルク');
disp(p2(1));

equation2 = sprintf('y2 = %.10fx',p2(1));
% text(-10,0.08,equation2,'Fontsize',12);

% 凡例の追加
legend('Measured Torque','Linear fit','Location','northwest','FontSize',12); % 凡例を追加、位置とフォントサイズを指定

% 軸ラベルの追加
xlabel('Input Values','FontSize',12); % X軸ラベル
ylabel('Torque [Nm]','FontSize',12);    % Y軸ラベル


grid on;
