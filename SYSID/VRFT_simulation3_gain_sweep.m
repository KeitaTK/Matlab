% VRFT_simulation3_gain_sweep.m
% 初期ゲイン（kp, kd）を自動で変更し、推定結果への影響を調査するスクリプト

clear; clc;

% 初期ゲインの範囲設定（roll軸のみ）
kp_init_list = linspace(0.1, 2.0, 5); % 0.1～2.0の間で5段階
kd_init_list = linspace(0.01, 0.5, 5); % 0.01～0.5の間で5段階

% 結果保存用
results = [];

fprintf('初期ゲイン変更による推定結果の調査開始...\n');
total = length(kp_init_list) * length(kd_init_list);
count = 0;

for kp_init = kp_init_list
    for kd_init = kd_init_list
        count = count + 1;
        fprintf('[%d/%d] kp_init=%.3f, kd_init=%.3f ... ', count, total, kp_init, kd_init);
        
        % VRFT推定関数を呼び出し（roll, pitch, yawの3軸）
        kp_init_vec = [kp_init; 16; 50];  % pitch, yawは固定
        kd_init_vec = [kd_init; 6; 5];
        
        [kp_est_vec, kd_est_vec] = estimate_vrft_gains(kp_init_vec, kd_init_vec);
        
        % roll軸のみ記録
        kp_est = kp_est_vec(1);
        kd_est = kd_est_vec(1);
        
        results = [results; kp_init, kd_init, kp_est, kd_est];
        fprintf('kp_est=%.6f, kd_est=%.6f\n', kp_est, kd_est);
    end
end

% 結果表示
fprintf('初期kp\t初期kd\t推定kp\t推定kd\n');
for i = 1:size(results,1)
    fprintf('%.3f\t%.3f\t%.3f\t%.3f\n', results(i,1), results(i,2), results(i,3), results(i,4));
end

% 結果をプロット
figure;
subplot(1,2,1);
scatter(results(:,1), results(:,3), 'filled');
xlabel('初期kp'); ylabel('推定kp'); title('初期kp vs 推定kp');
subplot(1,2,2);
scatter(results(:,2), results(:,4), 'filled');
xlabel('初期kd'); ylabel('推定kd'); title('初期kd vs 推定kd');
sgtitle('初期ゲイン変更による推定結果の変化');
