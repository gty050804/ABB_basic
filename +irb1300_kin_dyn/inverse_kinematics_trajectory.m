function [q_traj, success] = inverse_kinematics_trajectory(robot, pose_traj, q_init, varargin)
%INVERSE_KINEMATICS_TRAJECTORY Sequential IK with previous solution as seed.
n_steps = size(pose_traj, 3);
q_traj = zeros(n_steps, robot.n_joints);
success = false(n_steps, 1);
q_curr = q_init(:);
for i = 1:n_steps
    [q_curr, success(i)] = irb1300_kin_dyn.inverse_kinematics( ...
        robot.screws, robot.m_home, pose_traj(:, :, i), q_curr, ...
        'q_lower', robot.q_lower, 'q_upper', robot.q_upper, varargin{:});
    q_traj(i, :) = q_curr.';
end
end
