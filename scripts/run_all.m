% Run all MATLAB IRB1300 kinematics/dynamics demos.
ROOT = fileparts(fileparts(mfilename('fullpath')));
scripts_dir = fullfile(ROOT, 'scripts');
script_names = {'01_forward_kinematics.m', '02_inverse_kinematics.m', ...
    '03_inverse_dynamics.m', '04_forward_dynamics.m', 'plot_results.m', ...
    '08_export_task3_task4_summaries.m'};

for i = 1:numel(script_names)
    fprintf('\n============================================================\n');
    fprintf('Running %s\n', script_names{i});
    fprintf('============================================================\n');
    run(fullfile(scripts_dir, script_names{i}));
end
fprintf('\nAll MATLAB demos completed. Results in results/.\n');
