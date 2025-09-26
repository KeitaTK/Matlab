clear;

dpick = py.DynPick.DynPick('COM3'); % ポート番号は適宜変更すること
dpick.show_firmware_version();
dpick.show_sensitivity();
disp(string(dpick.read_temperature())+'(deg C)');
data = dpick.read_once();

dpick.start_continuous_read();
sample_time=zeros(3001,1);
tic;
get_data=zeros(3001,6);

for v=1:1:3000
    data = dpick.read_continuous();
    pause(0.01);
    sample_time(v+1) = toc;
    get_data(v+1) = double(data);
    disp(v);
end
toc


save_data = [sample_time,get_data];
save("save_data", "sample_time");


x1=save_data(:,1);
y1=-save_data(:,4);

p1 = polyfit(x1, y1, 1);
x_fit1 = linspace(min(x1), max(x1),100);
y_fit1 = polyval(p1, x_fit1);
figure(1);
plot(x1,y1,'-o',x_fit1,y_fit1);

xlavel('時間');
ylavel('z方向の推力');
grid on;
disp(double(data));
disp("[t], [N]");

x2=save_data(:,1);
y2=save_data(:,7);

p2 = polyfit(x2, y2, 1);
x_fit2 = linspace(min(x2), max(x2),100);
y_fit2 = polyval(p2, x_fit2);
figure(2);
plot(x2,y2,'-o',x_fit2,y_fit2);

xlavel('時間');
ylavel('トルク');
grid on;
disp(double(data));
disp("[t], [Nm]");