clear;

T = readtable('output45.csv'); % テーブルを読み込む

% 配列に変換
time = table2array(T(:,1));
euler = table2array(T(:, [2 3 4]))*180/pi();
motor = table2array(T(:, [5 6 7 8]));

deuler = zeros(length(time)-1,3);

for j = 1:3
    for i = 1:(length(time)-1)
        deuler(i,j) = euler(i+1,j) - euler(i,j);
    end
end

% % 微分値(deuler)にローパスフィルタをかける。
deuler_lowpass = zeros(size(deuler));
for i = 1:3
    deuler_lowpass(:,i) = lowpass(deuler(:,i),2,100);
end    


figure(3);clf; hold('on');
plot(time(1:length(time)-1), deuler_lowpass(:,1));
plot(time(1:length(time)-1), deuler_lowpass(:,2));
plot(time(1:length(time)-1), deuler_lowpass(:,3));
hold off;

figure(2);clf; hold('on');
plot(time(1:length(time)-1), deuler(:,1));
plot(time(1:length(time)-1), deuler(:,2));
plot(time(1:length(time)-1), deuler(:,3));
hold off;
% plot(time, deuler(:,2));
% plot(time, deuler(:,3));
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

