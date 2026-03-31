function signal = generate_signal_highfreq_interference(varargin)
% generate_signal_highfreq_interference
% Generates low-frequency target + high-frequency interference + optional noise.

p = inputParser;
addParameter(p, 'fs', 100);
addParameter(p, 'duration', 60);
addParameter(p, 'f_target', 0.8);
addParameter(p, 'f_interf', 5.0);
addParameter(p, 'amp_target', 1.0);
addParameter(p, 'amp_ratio_interf', 0.3);
addParameter(p, 'offset', 0.0);
addParameter(p, 'phase_target', 0.0);
addParameter(p, 'phase_interf', 0.0);
addParameter(p, 'decay', 0.0);
addParameter(p, 'noise_std', 0.01);
addParameter(p, 'seed', 1);
addParameter(p, 'outFile', '');
parse(p, varargin{:});

cfg = p.Results;
rng(cfg.seed);

dt = 1 / cfg.fs;
t = (0:dt:cfg.duration)';
env = exp(-cfg.decay * t);

y_target = cfg.amp_target .* env .* sin(2 * pi * cfg.f_target * t + cfg.phase_target);
y_interf = (cfg.amp_target * cfg.amp_ratio_interf) .* env .* sin(2 * pi * cfg.f_interf * t + cfg.phase_interf);
y_clean = y_target + y_interf + cfg.offset;
y = y_clean + cfg.noise_std .* randn(size(t));

signal = struct();
signal.t = t;
signal.y = y;
signal.y_clean = y_clean;
signal.y_target = y_target;
signal.y_interf = y_interf;
signal.true_freq_hz = cfg.f_target * ones(size(t));
signal.true_omega = 2 * pi * signal.true_freq_hz;
signal.meta = cfg;

if ~isempty(cfg.outFile)
    T = table(t, y, y_clean, y_target, y_interf, signal.true_freq_hz, ...
        'VariableNames', {'Time_s','Signal','SignalClean','Target','Interference','TrueFreq_Hz'});
    writetable(T, cfg.outFile);
    signal.saved_path = cfg.outFile;
else
    signal.saved_path = '';
end
end
