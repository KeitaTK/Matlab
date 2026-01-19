function out = generate_multi_frequency_sine(varargin)
% generate_multi_frequency_sine Generate multi-frequency damped sine signal
% Usage:
%   out = generate_multi_frequency_sine()  % use defaults
%   out = generate_multi_frequency_sine('fs',100,'duration',60,...)

p = inputParser;
addParameter(p,'fs',100);
addParameter(p,'duration',60);
addParameter(p,'f_primary',0.8);      % Primary frequency (target)
addParameter(p,'f_secondary',5.0);    % Secondary frequency (interference)
addParameter(p,'A_primary',1.0);      % Primary amplitude
addParameter(p,'A_secondary',0.3);    % Secondary amplitude
addParameter(p,'decay',0.01);         % 1/s
addParameter(p,'phi0_primary',0.0);
addParameter(p,'phi0_secondary',0.0);
addParameter(p,'noise_std',0.01);
addParameter(p,'outname','multi_freq_08hz_5hz_60s.csv');
parse(p,varargin{:});

fs = p.Results.fs;
duration = p.Results.duration;
f_primary = p.Results.f_primary;
f_secondary = p.Results.f_secondary;
A_primary = p.Results.A_primary;
A_secondary = p.Results.A_secondary;
decay = p.Results.decay;
phi0_primary = p.Results.phi0_primary;
phi0_secondary = p.Results.phi0_secondary;
noise_std = p.Results.noise_std;
outname = p.Results.outname;

dt = 1/fs;
t = (0:dt:duration)';

% Amplitude envelope (applies to both)
amp = exp(-decay .* t);

% Primary component (target frequency)
phase_primary = 2*pi*f_primary .* t + phi0_primary;
y_primary = A_primary .* amp .* sin(phase_primary);

% Secondary component (interference frequency)
phase_secondary = 2*pi*f_secondary .* t + phi0_secondary;
y_secondary = A_secondary .* amp .* sin(phase_secondary);

% Combined signal
y = y_primary + y_secondary + noise_std .* randn(size(t));

% Create output table
T = table(t, y, y_primary, y_secondary, amp, ...
          repmat(f_primary,size(t)), repmat(f_secondary,size(t)));
T.Properties.VariableNames = {'Time_s','Signal','Primary','Secondary',...
                               'TrueAmp','TrueFreq_Primary_Hz','TrueFreq_Secondary_Hz'};

% Write to data directory
script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir,'..','data');
if ~exist(data_dir,'dir'), mkdir(data_dir); end
outpath = fullfile(data_dir, outname);

writetable(T, outpath);

out.t = t;
out.y = y;
out.y_primary = y_primary;
out.y_secondary = y_secondary;
out.amp = amp;
out.path = outpath;
fprintf('Generated multi-frequency signal and saved to: %s\n', outpath);
fprintf('  Primary: %.2f Hz (A=%.2f)\n', f_primary, A_primary);
fprintf('  Secondary: %.2f Hz (A=%.2f)\n', f_secondary, A_secondary);
end
