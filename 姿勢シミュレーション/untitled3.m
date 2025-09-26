T = readtable('output45.csv');

time_data = T{:, 1};
value_data = T{:, 2:end};

% 微分値を計算 (前述のコードと同様)
diff_value = zeros(size(value_data));
for j = 1:size(value_data, 2)
    for i = 2:length(time_data)-1
        diff_value(i, j) = (value_data(i+1, j) - value_data(i-1, j)) / (time_data(i+1) - time_data(i-1));
    end
    diff_value(1,j) = (value_data(2, j) - value_data(1, j)) / (time_data(2) - time_data(1));
    diff_value(end,j) = (value_data(end, j) - value_data(end-1, j)) / (time_data(end) - time_data(end-1));
end

% 移動平均フィルタを適用
window_size = 5; % 移動平均の窓幅 (奇数にするのが一般的)
filtered_diff_value = zeros(size(diff_value));
for j = 1:size(diff_value, 2)
    filtered_diff_value(:,j) = movmean(diff_value(:,j), window_size);
end

% 時刻が17の時のフィルタ後の微分値を求める (前述のコードと同様)
target_time = 17;
[~, target_index] = min(abs(time_data - target_time));
filtered_diff_value_at_target_time = filtered_diff_value(target_index, :);

disp(['時刻 ', num2str(target_time), ' の時のフィルタ後の微分値 (移動平均): ', num2str(filtered_diff_value_at_target_time)]);

figure;
hold on
plot(time_data,value_data(:,1));
plot(time_data,diff_value(:,1));
plot(time_data,filtered_diff_value(:,1));
legend('Original Value','Derivative','Filtered Derivative');
hold off