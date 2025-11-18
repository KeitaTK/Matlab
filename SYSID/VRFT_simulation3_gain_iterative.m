% VRFT_simulation3_gain_iterative.m
% 推定値を次の初期値に設定し、ゲインが収束するまで繰り返すスクリプト

clear; clc;

% 初期値（任意に設定、roll軸のみ変更）
kp_init = 1.0;
kd_init = 0.1;

% 収束判定用パラメータ
max_iter = 50;
tol = 1e-4; % 収束判定閾値

results = [];
fprintf('ゲイン収束計算開始...\n');

for iter = 1:max_iter
    fprintf('[iter %d] kp_init=%.6f, kd_init=%.6f ... ', iter, kp_init, kd_init);
    
    % VRFT推定関数を呼び出し（roll, pitch, yawの3軸）
    kp_init_vec = [kp_init; 16; 50];  % pitch, yawは固定
    kd_init_vec = [kd_init; 6; 5];
    
    [kp_est_vec, kd_est_vec] = estimate_vrft_gains(kp_init_vec, kd_init_vec);
    
    % roll軸のみ記録
    kp_est = kp_est_vec(1);
    kd_est = kd_est_vec(1);
    
    results = [results; iter, kp_init, kd_init, kp_est, kd_est];
    fprintf('kp_est=%.6f, kd_est=%.6f\n', kp_est, kd_est);
    
    % 収束判定
    if iter > 1
        if abs(kp_est - kp_init) < tol && abs(kd_est - kd_init) < tol
            fprintf('収束しました（%d回目）\n', iter);
            break;
        end
    end
    
    % 次の初期値に推定値を設定
    kp_init = kp_est;
    kd_init = kd_est;
end

% 結果表示
fprintf('iter\t初期kp\t初期kd\t推定kp\t推定kd\n');
for i = 1:size(results,1)
    fprintf('%d\t%.6f\t%.6f\t%.6f\t%.6f\n', results(i,1), results(i,2), results(i,3), results(i,4), results(i,5));
end

% 収束過程をプロット
figure;
subplot(1,2,1);
plot(results(:,1), results(:,4), '-o');
xlabel('iteration'); ylabel('推定kp'); title('推定kpの収束過程');
subplot(1,2,2);
plot(results(:,1), results(:,5), '-o');
xlabel('iteration'); ylabel('推定kd'); title('推定kdの収束過程');
sgtitle('ゲイン推定値の収束過程');
