function signal = generate_signal_baseline(varargin)
% generate_signal_baseline
% Generates a baseline sinusoidal disturbance with DC offset.

p = inputParser;
addParameter(p, 'fs', 100);
addParameter(p, 'duration', 60);
addParameter(p, 'f_true', 0.8);
addParameter(p, 'amplitude', 1.0);
addParameter(p, 'offset', 0.0);
addParameter(p, 'phase0', 0.0);
addParameter(p, 'decay', 0.0);
addParameter(p, 'noise_std', 0.0);
addParameter(p, 'seed', 1);
addParameter(p, 'outFile', '');
parse(p, varargin{:});

cfg = p.Results;
rng(cfg.seed);

dt = 1 / cfg.fs;
t = (0:dt:cfg.duration)';
env = exp(-cfg.decay * t);

y_clean = cfg.amplitude .* env .* sin(2 * pi * cfg.f_true * t + cfg.phase0) + cfg.offset;
y = y_clean + cfg.noise_std .* randn(size(t));

signal = struct();
signal.t = t;
signal.y = y;
signal.y_clean = y_clean;
signal.true_freq_hz = cfg.f_true * ones(size(t));
signal.true_omega = 2 * pi * signal.true_freq_hz;
signal.components = table(t, y_clean, env, 'VariableNames', {'Time_s','Baseline','Envelope'});
signal.meta = cfg;

if ~isempty(cfg.outFile)
    T = table(t, y, y_clean, signal.true_freq_hz, ...
        'VariableNames', {'Time_s','Signal','SignalClean','TrueFreq_Hz'});
    writetable(T, cfg.outFile);
    signal.saved_path = cfg.outFile;
else
    signal.saved_path = '';
end
end
