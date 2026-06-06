% Export readable Task 3/4 theoretical input-output summaries.
ROOT = fileparts(fileparts(mfilename('fullpath')));
addpath(ROOT);

results_dir = fullfile(ROOT, 'results');
output_dir = fullfile(results_dir, 'summaries');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

task3_path = fullfile(output_dir, 'task3_inverse_dynamics_summary.md');
task4_path = fullfile(output_dir, 'task4_forward_dynamics_summary.md');
write_text(task3_path, task3_summary(results_dir));
write_text(task4_path, task4_summary(results_dir));
fprintf('Saved %s\n', task3_path);
fprintf('Saved %s\n', task4_path);

function text = task3_summary(results_dir)
d = load(fullfile(results_dir, 'inverse_dynamics.mat'));
lines = ["# Task 3 Inverse Dynamics Summary"; ""; ...
    "MATLAB theoretical recursive Newton-Euler results."; ""; ...
    "## Inputs"; ""; ...
    "- Joint order: `" + strjoin(string(d.joint_names), ", ") + "`"; ...
    "- End-effector wrench `[Fx,Fy,Fz,Mx,My,Mz]`: `" + vec(d.f_ext_ee) + "`"; ...
    "- Number of trajectory steps: `" + string(size(d.q_traj, 1)) + "`"; ""; ...
    "## Error Check"; ""; ...
    "- `max(norm(tau_total - tau_drive - tau_constraint)) = " + ...
    string(max(vecnorm(d.tau_total - d.tau_drive - d.tau_constraint, 2, 2))) + "`"; ""];
for i = 1:size(d.q_traj, 1)
    lines(end+1) = "### Step " + string(i - 1); %#ok<AGROW>
    lines(end+1) = "- q: `" + vec(d.q_traj(i, :)) + "`"; %#ok<AGROW>
    lines(end+1) = "- qd: `" + vec(d.qd_traj(i, :)) + "`"; %#ok<AGROW>
    lines(end+1) = "- qdd: `" + vec(d.qdd_traj(i, :)) + "`"; %#ok<AGROW>
    lines(end+1) = "- tau_drive: `" + vec(d.tau_drive(i, :)) + "`"; %#ok<AGROW>
    lines(end+1) = "- tau_constraint: `" + vec(d.tau_constraint(i, :)) + "`"; %#ok<AGROW>
    lines(end+1) = "- tau_total: `" + vec(d.tau_total(i, :)) + "`"; %#ok<AGROW>
    lines(end+1) = ""; %#ok<AGROW>
end
text = join(lines, newline) + newline;
end

function text = task4_summary(results_dir)
d = load(fullfile(results_dir, 'forward_dynamics.mat'));
lines = ["# Task 4 Forward Dynamics Summary"; ""; ...
    "MATLAB theoretical forward dynamics result."; ""; ...
    "- q [rad]: `" + vec(d.q) + "`"; ...
    "- qd [rad/s]: `" + vec(d.qd) + "`"; ...
    "- drive torque [Nm]: `" + vec(d.tau_drive) + "`"; ...
    "- end-effector wrench `[Fx,Fy,Fz,Mx,My,Mz]`: `" + vec(d.f_ext_ee) + "`"; ...
    "- target qdd [rad/s^2]: `" + vec(d.qdd_target) + "`"; ...
    "- calculated qdd [rad/s^2]: `" + vec(d.qdd) + "`"; ...
    "- target error norm: `" + string(norm(d.qdd - d.qdd_target)) + "`"; ""];
text = join(lines, newline) + newline;
end

function s = vec(v)
v = v(:).';
parts = compose('%.6g', v);
s = "[" + strjoin(parts, ", ") + "]";
end

function write_text(path, text)
fid = fopen(path, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s', text);
end
