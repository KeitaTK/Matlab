% analyze_sweep_results.m - Analyze all sweep results and generate summary
clear; close all; clc;

results_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');

% Check if v2_all_sweep_results.mat exists
mat_file = fullfile(results_dir, 'v2_all_sweep_results.mat');
if exist(mat_file, 'file')
    fprintf('Loading v2_all_sweep_results.mat...\n');
    load(mat_file);
    
    % Display structure
    whos
    
    % Save summary to text file
    summary_file = fullfile(results_dir, 'sweep_summary.txt');
    fid = fopen(summary_file, 'w');
    
    fprintf(fid, '=== RLS Phase Estimation - Comprehensive Sweep Results ===\n\n');
    
    % List all variables in the workspace
    vars = who;
    for i = 1:length(vars)
        var_name = vars{i};
        var_data = eval(var_name);
        fprintf(fid, '\nVariable: %s\n', var_name);
        fprintf(fid, 'Type: %s\n', class(var_data));
        fprintf(fid, 'Size: %s\n', mat2str(size(var_data)));
        
        % If it's a struct, show fields
        if isstruct(var_data)
            fields = fieldnames(var_data);
            fprintf(fid, 'Fields: %s\n', strjoin(fields, ', '));
            
            % Display some key metrics if available
            if isfield(var_data, 'conv_time')
                fprintf(fid, '  Convergence times: %s\n', mat2str(var_data.conv_time));
            end
            if isfield(var_data, 'steady_error')
                fprintf(fid, '  Steady errors: %s\n', mat2str(var_data.steady_error));
            end
            if isfield(var_data, 'steady_std')
                fprintf(fid, '  Steady std: %s\n', mat2str(var_data.steady_std));
            end
        end
    end
    
    fclose(fid);
    fprintf('Summary saved to: %s\n', summary_file);
    
    % Display in console
    type(summary_file);
else
    fprintf('v2_all_sweep_results.mat not found.\n');
    fprintf('Searching for individual sweep files...\n\n');
    
    % Load individual sweep MAT files
    sweep_files = {
        'v2_sweep_alpha.mat', 'alpha';
        'v2_sweep_decay.mat', 'decay';
        'v2_sweep_gamma.mat', 'gamma';
        'v2_sweep_init_omega.mat', 'init_omega';
        'v2_sweep_noise.mat', 'noise'
    };
    
    summary = struct();
    
    for i = 1:size(sweep_files, 1)
        fname = sweep_files{i, 1};
        label = sweep_files{i, 2};
        fpath = fullfile(results_dir, fname);
        
        if exist(fpath, 'file')
            fprintf('Loading %s...\n', fname);
            data = load(fpath);
            summary.(label) = data;
            
            % Display info
            fprintf('  Fields: %s\n', strjoin(fieldnames(data), ', '));
            if isfield(data, 'results')
                res = data.results;
                if isfield(res, 'conv_time')
                    fprintf('  Convergence times: %s\n', mat2str(res.conv_time'));
                end
                if isfield(res, 'steady_error')
                    fprintf('  Steady errors (mean): %.6f\n', mean(abs(res.steady_error)));
                end
            end
        else
            fprintf('  File not found: %s\n', fname);
        end
    end
    
    % Save combined summary
    save(fullfile(results_dir, 'combined_summary.mat'), 'summary');
    fprintf('\nCombined summary saved to combined_summary.mat\n');
end

fprintf('\n=== Analysis complete ===\n');
