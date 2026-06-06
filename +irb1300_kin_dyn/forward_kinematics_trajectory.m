function pose_traj = forward_kinematics_trajectory(robot, q_traj)
%FORWARD_KINEMATICS_TRAJECTORY Evaluate FK for an N-by-n trajectory.
n_steps = size(q_traj, 1);
pose_traj = zeros(4, 4, n_steps);
for i = 1:n_steps
    pose_traj(:, :, i) = irb1300_kin_dyn.forward_kinematics(robot, q_traj(i, :).');
end
end
