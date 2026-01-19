% generate_detailed_analysis.m - Generate detailed analysis report
clear; close all; clc;

results_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
mat_file = fullfile(results_dir, 'v2_all_sweep_results.mat');

fprintf('Loading results...\n');
load(mat_file);

% Initialize report data
report = struct();
report.timestamp = datetime('now');
report.sweeps = {};

% Process each sweep
for s = 1:length(all_results)
    sweep_data = all_results{s};
    
    fprintf('\nProcessing sweep %d: %s\n', s, sweep_data.sweep_name);
    
    % Extract data
    sweep_info = struct();
    sweep_info.name = sweep_data.sweep_name;
    sweep_info.param_name = sweep_data.param_name;
    sweep_info.values = sweep_data.values;
    sweep_info.conv_time = sweep_data.conv_time;
    sweep_info.steady_error = sweep_data.steady_error;
    sweep_info.steady_std = sweep_data.steady_std;
    sweep_info.final_freq = sweep_data.final_freq;
    
    % Calculate statistics
    sweep_info.best_conv_idx = find(sweep_data.conv_time == min(sweep_data.conv_time), 1);
    sweep_info.best_conv_value = sweep_data.values(sweep_info.best_conv_idx);
    sweep_info.best_conv_time = sweep_data.conv_time(sweep_info.best_conv_idx);
    
    sweep_info.best_error_idx = find(abs(sweep_data.steady_error) == min(abs(sweep_data.steady_error)), 1);
    sweep_info.best_error_value = sweep_data.values(sweep_info.best_error_idx);
    sweep_info.best_error = sweep_data.steady_error(sweep_info.best_error_idx);
    
    sweep_info.best_std_idx = find(sweep_data.steady_std == min(sweep_data.steady_std), 1);
    sweep_info.best_std_value = sweep_data.values(sweep_info.best_std_idx);
    sweep_info.best_std = sweep_data.steady_std(sweep_info.best_std_idx);
    
    report.sweeps{s} = sweep_info;
    
    % Display summary
    fprintf('  Parameter: %s\n', sweep_info.param_name);
    fprintf('  Values tested: %s\n', mat2str(sweep_info.values));
    fprintf('  Best convergence: %.2f s at %s = %g\n', ...
        sweep_info.best_conv_time, sweep_info.param_name, sweep_info.best_conv_value);
    fprintf('  Best error: %.6f Hz at %s = %g\n', ...
        sweep_info.best_error, sweep_info.param_name, sweep_info.best_error_value);
    fprintf('  Best std: %.6f Hz at %s = %g\n', ...
        sweep_info.best_std, sweep_info.param_name, sweep_info.best_std_value);
end

% Save detailed report
save(fullfile(results_dir, 'detailed_analysis.mat'), 'report');
fprintf('\nDetailed analysis saved to detailed_analysis.mat\n');

% Generate text report
txt_file = fullfile(results_dir, 'detailed_report.txt');
fid = fopen(txt_file, 'w');

fprintf(fid, '========================================\n');
fprintf(fid, 'RLS Phase Estimation - Detailed Analysis\n');
fprintf(fid, '========================================\n\n');
fprintf(fid, 'Generated: %s\n\n', char(report.timestamp));

for s = 1:length(report.sweeps)
    sweep = report.sweeps{s};
    
    fprintf(fid, '\n--- Sweep %d: %s ---\n', s, sweep.name);
    fprintf(fid, 'Parameter: %s\n', sweep.param_name);
    fprintf(fid, 'Values tested: %s\n\n', mat2str(sweep.values));
    
    fprintf(fid, 'Results Table:\n');
    fprintf(fid, '%15s | %12s | %15s | %15s | %15s\n', ...
        sweep.param_name, 'Conv [s]', 'Error [Hz]', 'Std [Hz]', 'Final [Hz]');
    fprintf(fid, '%s\n', repmat('-', 1, 80));
    
    for i = 1:length(sweep.values)
        fprintf(fid, '%15g | %12.4f | %15.8f | %15.8f | %15.8f\n', ...
            sweep.values(i), sweep.conv_time(i), sweep.steady_error(i), ...
            sweep.steady_std(i), sweep.final_freq(i));
    end
    
    fprintf(fid, '\nBest Performance:\n');
    fprintf(fid, '  Fastest convergence: %.2f s at %s = %g\n', ...
        sweep.best_conv_time, sweep.param_name, sweep.best_conv_value);
    fprintf(fid, '  Lowest error: %.8f Hz at %s = %g\n', ...
        sweep.best_error, sweep.param_name, sweep.best_error_value);
    fprintf(fid, '  Lowest std: %.8f Hz at %s = %g\n', ...
        sweep.best_std, sweep.param_name, sweep.best_std_value);
end

fprintf(fid, '\n\n========================================\n');
fprintf(fid, 'End of Report\n');
fprintf(fid, '========================================\n');

fclose(fid);

fprintf('\nText report saved to: %s\n', txt_file);
type(txt_file);

fprintf('\n=== Analysis complete ===\n');
