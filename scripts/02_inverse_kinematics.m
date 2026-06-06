% Demo 2: Inverse kinematics - end-effector pose trajectory to joints.
ROOT = fileparts(fileparts(mfilename('fullpath')));
addpath(ROOT);

steps = 30;
output = fullfile(ROOT, 'results', 'inverse_kinematics.mat');
robot = irb1300_kin_dyn.create_robot();
q_init = irb1300_kin_dyn.default_home_configuration(robot.n_joints);

[q_traj_ref, ~, ~] = irb1300_kin_dyn.joint_sinusoid_trajectory(steps, robot.n_joints, 0.05, ...
    'amplitude', 0.15, 'q_center', q_init);
pose_traj = irb1300_kin_dyn.forward_kinematics_trajectory(robot, q_traj_ref);
[q_traj_ik, ik_success] = irb1300_kin_dyn.inverse_kinematics_trajectory(robot, pose_traj, q_init);
fk_check = irb1300_kin_dyn.forward_kinematics_trajectory(robot, q_traj_ik);

position_error = zeros(steps, 1);
for i = 1:steps
    position_error(i) = norm(fk_check(1:3, 4, i) - pose_traj(1:3, 4, i));
end
ik_success = position_error < 1e-3;

if ~exist(fileparts(output), 'dir')
    mkdir(fileparts(output));
end
joint_names = robot.joint_names; %#ok<NASGU>
save(output, 'pose_traj', 'q_traj_ref', 'q_traj_ik', 'ik_success', 'position_error', 'joint_names');
fprintf('IK success rate: %.1f%%\n', mean(ik_success) * 100);
fprintf('Mean position error: %.3e m\n', mean(position_error));
fprintf('Max position error: %.3e m\n', max(position_error));
fprintf('Saved to %s\n', output);
