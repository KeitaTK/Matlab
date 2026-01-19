function [t, y, params] = generate_damped_sine(fs, duration, f_true, A_true, decay, noise_amp)
% GENERATE_DAMPED_SINE 減衰正弦信号を生成
%
% 入力:
%   fs        - サンプリング周波数 [Hz]
%   duration  - 信号長 [s]
%   f_true    - 真の周波数 [Hz]
%   A_true    - 真の振幅
%   decay     - 減衰係数 (例: 0.01 で約 30 秒の時定数)
%   noise_amp - ノイズ振幅 (0 でノイズなし)
%
% 出力:
%   t      - 時刻ベクトル (N x 1)
%   y      - 観測信号 (N x 1)
%   params - 真値構造体 (.A, .B, .C, .omega, .freq)

% 時刻ベクトル
t = (0 : 1/fs : duration - 1/fs)';
N = length(t);

% 真の角周波数
omega_true = 2 * pi * f_true;

% 真値パラメータ（初期値）
A_true_init = A_true;
B_true_init = 0;
C_true_init = 0;

% 減衰正弦波生成
envelope = exp(-decay * t);
y_clean = envelope .* (A_true_init * sin(omega_true * t) + B_true_init * cos(omega_true * t)) + C_true_init;

% ノイズ付加
noise = noise_amp * randn(N, 1);
y = y_clean + noise;

% 真値を構造体で返す
params.A = A_true_init;
params.B = B_true_init;
params.C = C_true_init;
params.omega = omega_true;
params.freq = f_true;
params.decay = decay;
params.noise_amp = noise_amp;

end
