% Run all MATLAB IRB1300 kinematics/dynamics demos.
ROOT = fileparts(fileparts(mfilename('fullpath')));
scripts_dir = fullfile(ROOT, 'scripts');
script_names = {'task01_forward_kinematics.m', 'task02_inverse_kinematics.m', ...
    'task03_inverse_dynamics.m', 'task04_forward_dynamics.m', 'plot_results.m', ...
    'task08_export_task3_task4_summaries.m'};

for i = 1:numel(script_names)
    fprintf('\n============================================================\n');
    fprintf('Running %s\n', script_names{i});
    fprintf('============================================================\n');
    run(fullfile(scripts_dir, script_names{i}));
end
fprintf('\nAll MATLAB demos completed. Results in results/.\n');
