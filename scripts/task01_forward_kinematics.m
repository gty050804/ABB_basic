% Demo 1: Forward kinematics - joint trajectory to end-effector pose.
ROOT = fileparts(fileparts(mfilename('fullpath')));
addpath(ROOT);

steps = 50;
output = fullfile(ROOT, 'results', 'forward_kinematics.mat');
robot = irb1300_kin_dyn.create_robot();

q_center = irb1300_kin_dyn.default_home_configuration(robot.n_joints);
[q_traj, ~, ~] = irb1300_kin_dyn.joint_sinusoid_trajectory(steps, robot.n_joints, 0.05, ...
    'q_center', q_center);
pose_traj = irb1300_kin_dyn.forward_kinematics_trajectory(robot, q_traj);

xyz_traj = zeros(steps, 3);
rpy_traj = zeros(steps, 3);
for i = 1:steps
    [xyz_traj(i, :), rpy_traj(i, :)] = irb1300_kin_dyn.transform_to_rpy_xyz(pose_traj(:, :, i));
end

if ~exist(fileparts(output), 'dir')
    mkdir(fileparts(output));
end
joint_names = robot.joint_names; %#ok<NASGU>
save(output, 'q_traj', 'pose_traj', 'xyz_traj', 'rpy_traj', 'joint_names');
fprintf('Saved %d FK samples to %s\n', steps, output);
fprintf('Sample end-effector position (last step): [%g %g %g]\n', xyz_traj(end, :));
