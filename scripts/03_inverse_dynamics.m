% Demo 3: Inverse dynamics - q, qd, qdd and EE wrench to torques.
ROOT = fileparts(fileparts(mfilename('fullpath')));
addpath(ROOT);

steps = 20;
output = fullfile(ROOT, 'results', 'inverse_dynamics.mat');
robot = irb1300_kin_dyn.create_robot();
q_center = irb1300_kin_dyn.default_home_configuration(robot.n_joints);
[q_traj, qd_traj, qdd_traj] = irb1300_kin_dyn.joint_sinusoid_trajectory(steps, robot.n_joints, 0.05, ...
    'amplitude', 0.2, 'q_center', q_center);

f_ext_ee = [0; 0; -10; 0; 0; 0];
tau_drive = zeros(steps, robot.n_joints);
tau_constraint = zeros(steps, robot.n_joints);
tau_total = zeros(steps, robot.n_joints);
for i = 1:steps
    result = irb1300_kin_dyn.inverse_dynamics(robot, q_traj(i, :).', qd_traj(i, :).', qdd_traj(i, :).', f_ext_ee);
    tau_drive(i, :) = result.tau_drive.';
    tau_constraint(i, :) = result.tau_constraint.';
    tau_total(i, :) = result.tau_total.';
end

if ~exist(fileparts(output), 'dir')
    mkdir(fileparts(output));
end
joint_names = robot.joint_names; %#ok<NASGU>
save(output, 'q_traj', 'qd_traj', 'qdd_traj', 'f_ext_ee', 'tau_drive', 'tau_constraint', 'tau_total', 'joint_names');
fprintf('External wrench at EE [N, Nm]: [%g %g %g %g %g %g]\n', f_ext_ee);
fprintf('Mean |tau_drive|: %.3f Nm\n', mean(vecnorm(tau_drive, 2, 2)));
fprintf('Mean |tau_constraint|: %.3f Nm\n', mean(vecnorm(tau_constraint, 2, 2)));
fprintf('Saved to %s\n', output);
