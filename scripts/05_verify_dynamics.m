% Verify Task 3 and Task 4 dynamics results.
ROOT = fileparts(fileparts(mfilename('fullpath')));
addpath(ROOT);

results_dir = fullfile(ROOT, 'results');
id_path = fullfile(results_dir, 'inverse_dynamics.mat');
fd_path = fullfile(results_dir, 'forward_dynamics.mat');
if ~isfile(id_path)
    error('Missing inverse_dynamics.mat. Run scripts/03_inverse_dynamics.m first.');
end
if ~isfile(fd_path)
    error('Missing forward_dynamics.mat. Run scripts/04_forward_dynamics.m first.');
end

robot = irb1300_kin_dyn.create_robot();
id = load(id_path);
fd = load(fd_path);
n = size(id.q_traj, 1);
decomp_err = zeros(n, 1);
eq_motion_err = zeros(n, 1);
wrench_err = zeros(n, 1);
id_recompute_err = zeros(n, 1);
id_fd_err = zeros(n, 1);

for i = 1:n
    q = id.q_traj(i, :).';
    qd = id.qd_traj(i, :).';
    qdd = id.qdd_traj(i, :).';
    decomp_err(i) = norm(id.tau_total(i, :).' - (id.tau_drive(i, :).' + id.tau_constraint(i, :).'));
    M = irb1300_kin_dyn.mass_matrix(robot, q);
    c = irb1300_kin_dyn.coriolis_gravity(robot, q, qd);
    eq_motion_err(i) = norm(id.tau_drive(i, :).' - (M * qdd + c));
    id_now = irb1300_kin_dyn.inverse_dynamics(robot, q, qd, qdd, id.f_ext_ee);
    wrench_err(i) = norm(id.tau_constraint(i, :).' - id_now.tau_constraint);
    id_recompute_err(i) = norm(id.tau_total(i, :).' - id_now.tau_total);
    qdd_fd = irb1300_kin_dyn.forward_dynamics(robot, q, qd, id.tau_total(i, :).', id.f_ext_ee);
    id_fd_err(i) = norm(qdd_fd - qdd);
end

qdd_fd = irb1300_kin_dyn.forward_dynamics(robot, fd.q, fd.qd, fd.tau_drive, fd.f_ext_ee);
fd_recompute_max = norm(qdd_fd - fd.qdd);
M = irb1300_kin_dyn.mass_matrix(robot, fd.q);
c = irb1300_kin_dyn.coriolis_gravity(robot, fd.q, fd.qd);
jtf = irb1300_kin_dyn.inverse_dynamics(robot, fd.q, fd.qd, zeros(robot.n_joints, 1), fd.f_ext_ee);
equation_max = norm(M * fd.qdd - (fd.tau_drive - c - jtf.tau_constraint));
id_total = irb1300_kin_dyn.inverse_dynamics(robot, fd.q, fd.qd, fd.qdd, fd.f_ext_ee);
id_total_consistency_max = norm(id_total.tau_total - (M * fd.qdd + c + jtf.tau_constraint));

fprintf('=== Dynamics verification ===\n\n');
fprintf('Task 3 - Inverse dynamics\n');
fprintf('  tau_total = tau_drive + tau_constraint max err: %.3e Nm\n', max(decomp_err));
fprintf('  tau_drive = M(q) qdd + C(q,qd) max err: %.3e Nm\n', max(eq_motion_err));
fprintf('  Wrench mapping recompute max err: %.3e Nm\n', max(wrench_err));
fprintf('  ID recompute max err: %.3e Nm\n', max(id_recompute_err));
fprintf('  ID -> FD roundtrip max err: %.3e rad/s^2\n\n', max(id_fd_err));
fprintf('Task 4 - Forward dynamics\n');
fprintf('  FD recompute max err: %.3e rad/s^2\n', fd_recompute_max);
fprintf('  Equation max err: %.3e\n', equation_max);
fprintf('  ID total consistency max err: %.3e Nm\n', id_total_consistency_max);

output = fullfile(results_dir, 'dynamics_verification.mat');
save(output, 'decomp_err', 'eq_motion_err', 'wrench_err', 'id_recompute_err', ...
    'id_fd_err', 'fd_recompute_max', 'equation_max', 'id_total_consistency_max');
fprintf('\nMetrics saved to %s\n', output);
