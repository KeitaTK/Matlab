function out = generate_damped_sine(varargin)
% generate_damped_sine Generate and save a damped sine signal (CSV)
% Usage:
%   out = generate_damped_sine()                 % use defaults
%   out = generate_damped_sine('fs',200,'duration',30,...)

p = inputParser;
addParameter(p,'fs',100);
addParameter(p,'duration',60);
addParameter(p,'f_true',0.8);
addParameter(p,'decay',0.1); % 1/s
addParameter(p,'A0',1.0);
addParameter(p,'phi0',0.0);
addParameter(p,'noise_std',0.0);
addParameter(p,'outname','damped_080hz_60s.csv');
parse(p,varargin{:});

fs = p.Results.fs;
duration = p.Results.duration;
f_true = p.Results.f_true;
decay = p.Results.decay;
A0 = p.Results.A0;
phi0 = p.Results.phi0;
noise_std = p.Results.noise_std;
outname = p.Results.outname;

dt = 1/fs;
t = (0:dt:duration)';
% amplitude envelope
amp = A0 .* exp(-decay .* t);
phase = 2*pi*f_true .* t + phi0;

y = amp .* sin(phase) + noise_std .* randn(size(t));

% create output table
T = table(t, y, amp, phase, repmat(f_true,size(t)));
T.Properties.VariableNames = {'Time_s','Signal','TrueAmp','TruePhase_rad','TrueFreq_Hz'};

% write to data directory (relative to this script)
script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir,'..','data');
if ~exist(data_dir,'dir'), mkdir(data_dir); end
outpath = fullfile(data_dir, outname);

writetable(T, outpath);

out.t = t; out.y = y; out.amp = amp; out.phase = phase; out.path = outpath;
fprintf('Generated damped sine and saved to: %s\n', outpath);
end