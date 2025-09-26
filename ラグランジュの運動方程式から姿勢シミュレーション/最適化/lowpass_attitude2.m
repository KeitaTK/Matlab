clear;

T = readtable('output44.csv'); % テーブルを読み込む

% 配列に変換
time = table2array(T(:,1));
euler = table2array(T(:, [2 3 4]))*180/pi();
motor = table2array(T(:, [5 6 7 8]));

euler_lowpass = zeros(size(euler));

for i = 1:3
    euler_lowpass(:,i) = lowpass(euler(:,i),20,100);
end


% figure(1);clf; hold('on');
% plot(time, euler(:,1));
% plot(time, euler(:,2));
% plot(time, euler(:,3));
% 
% figure(2);clf; hold('on');
% plot(time, motor(:,1));
% plot(time, motor(:,2));
% plot(time, motor(:,3));
% plot(time, motor(:,4));


figure(1); clf;
subplot(2,1,1); % 2行1列の1番目
plot(time, euler(:,1));
hold on;
plot(time, euler(:,2));
plot(time, euler(:,3));
title('Euler Angles');
xlabel('時間 (秒)');
ylabel('姿勢角(度)');


subplot(2,1,2); % 2行1列の2番目
plot(time, motor(:,1));
hold on;
plot(time, motor(:,2));
plot(time, motor(:,3));
plot(time, motor(:,4));
title('Motor Outputs');

figure(2); clf;
subplot(2,1,1); % 2行1列の1番目
plot(time, euler(:,2));
hold on;
plot(time, euler_lowpass(:,2));