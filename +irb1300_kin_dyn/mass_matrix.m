function M = mass_matrix(robot, q)
%MASS_MATRIX Build the joint-space mass matrix using inverse dynamics.
n = robot.n_joints;
M = zeros(n, n);
zero = zeros(n, 1);
for i = 1:n
    qdd = zeros(n, 1);
    qdd(i) = 1;
    M(:, i) = irb1300_kin_dyn.inverse_dynamics_raw(robot, q, zero, qdd, zeros(3, 1), zeros(6, 1));
end
end
