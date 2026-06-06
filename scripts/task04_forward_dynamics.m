% Demo 4: Forward dynamics - q, qd, tau and EE wrench to joint accelerations.
ROOT = fileparts(fileparts(mfilename('fullpath')));
addpath(ROOT);

output = fullfile(ROOT, 'results', 'forward_dynamics.mat');
robot = irb1300_kin_dyn.create_robot();
q = irb1300_kin_dyn.default_home_configuration(robot.n_joints).';
qd = [0.1; -0.05; 0.08; 0; 0.06; -0.04];
f_ext_ee = [0; 0; -8; 0; 0; 0];
qdd_target = [0.5; -0.3; 0.4; 0.2; -0.2; 0.3];
id_target = irb1300_kin_dyn.inverse_dynamics(robot, q, qd, qdd_target, f_ext_ee);
tau_drive = id_target.tau_total;
qdd = irb1300_kin_dyn.forward_dynamics(robot, q, qd, tau_drive, f_ext_ee);
id_check = irb1300_kin_dyn.inverse_dynamics(robot, q, qd, qdd, f_ext_ee);
residual = norm(id_check.tau_total - (id_check.tau_drive + id_check.tau_constraint));

if ~exist(fileparts(output), 'dir')
    mkdir(fileparts(output));
end
joint_names = robot.joint_names; %#ok<NASGU>
tau_total_check = id_check.tau_total; %#ok<NASGU>
save(output, 'q', 'qd', 'qdd', 'qdd_target', 'tau_drive', 'f_ext_ee', 'tau_total_check', 'joint_names');
fprintf('Joint positions q: [%s]\n', num2str(q.'));
fprintf('Joint velocities qd: [%s]\n', num2str(qd.'));
fprintf('Driving torques: [%s]\n', num2str(tau_drive.'));
fprintf('Computed joint accelerations qdd: [%s]\n', num2str(qdd.'));
fprintf('ID consistency residual: %.3e\n', residual);
fprintf('Saved to %s\n', output);
