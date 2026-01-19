function [y, t, true_omega_primary, y_primary, y_secondary] = generate_multi_frequency_sine(...
    f_primary, f_secondary, A_primary, A_secondary, decay_rate, noise_std, T, fs, filename)
% GENERATE_MULTI_FREQUENCY_SINE  Generates multi-frequency damped sine signal
%
% Inputs:
%   f_primary      - Primary frequency (target) [Hz]
%   f_secondary    - Secondary frequency (interference) [Hz]
%   A_primary      - Primary amplitude
%   A_secondary    - Secondary amplitude (interference)
%   decay_rate     - Decay rate [1/s]
%   noise_std      - Noise standard deviation
%   T              - Duration [s]
%   fs             - Sampling frequency [Hz]
%   filename       - Output CSV filename (optional)
%
% Outputs:
%   y              - Combined signal (N x 1)
%   t              - Time vector (N x 1) [s]
%   true_omega_primary - True primary angular frequency (N x 1) [rad/s]
%   y_primary      - Primary component only (N x 1)
%   y_secondary    - Secondary component only (N x 1)

dt = 1 / fs;
t = (0:dt:T)';
N = length(t);

% Amplitude envelope (exponential decay)
amp = exp(-decay_rate * t);

% Primary component (target frequency)
omega_primary = 2 * pi * f_primary;
phase_primary = omega_primary * t;
y_primary = A_primary * amp .* sin(phase_primary);

% Secondary component (interference frequency)
omega_secondary = 2 * pi * f_secondary;
phase_secondary = omega_secondary * t;
y_secondary = A_secondary * amp .* sin(phase_secondary);

% Combined signal with noise
y = y_primary + y_secondary + noise_std * randn(N, 1);

% True primary angular frequency (constant in this case)
true_omega_primary = omega_primary * ones(N, 1);

% Save to CSV if filename provided
if nargin >= 9 && ~isempty(filename)
    % Create data directory if it doesn't exist
    [filepath, ~, ~] = fileparts(filename);
    if ~exist(filepath, 'dir')
        mkdir(filepath);
    end
    
    % Create table and write
    data_table = table(t, y, y_primary, y_secondary, amp, ...
        repmat(f_primary, N, 1), repmat(f_secondary, N, 1));
    data_table.Properties.VariableNames = {'Time_s', 'Signal', 'Primary', ...
        'Secondary', 'Amplitude', 'TrueFreq_Primary_Hz', 'TrueFreq_Secondary_Hz'};
    writetable(data_table, filename);
    
    fprintf('Generated multi-frequency signal and saved to: %s\n', filename);
    fprintf('  Primary: %.2f Hz (A=%.2f)\n', f_primary, A_primary);
    fprintf('  Secondary: %.2f Hz (A=%.2f)\n', f_secondary, A_secondary);
end

end
