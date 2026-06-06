function tau = inverse_dynamics_raw(robot, q, qd, qdd, gravity, ftip)
%INVERSE_DYNAMICS_RAW Recursive Newton-Euler inverse dynamics core.
n = robot.n_joints;
q = q(:); qd = qd(:); qdd = qdd(:);
transforms = zeros(4, 4, n + 2);
for i = 1:n+2
    transforms(:, :, i) = eye(4);
end
velocities = zeros(6, n + 1);
accelerations = zeros(6, n + 1);
accelerations(4:6, 1) = -gravity(:);

for i = 1:n
    screw = robot.alist(:, i);
    transforms(:, :, i + 1) = irb1300_kin_dyn.matrix_exp_se3(-screw, q(i)) * ...
        irb1300_kin_dyn.transform_inverse(robot.mlist(:, :, i));
    velocities(:, i + 1) = irb1300_kin_dyn.adjoint(transforms(:, :, i + 1)) * velocities(:, i) + screw * qd(i);
    accelerations(:, i + 1) = irb1300_kin_dyn.adjoint(transforms(:, :, i + 1)) * accelerations(:, i) + ...
        irb1300_kin_dyn.ad_twist(velocities(:, i + 1)) * (screw * qd(i)) + screw * qdd(i);
end

transforms(:, :, n + 2) = irb1300_kin_dyn.transform_inverse(robot.mlist(:, :, n + 1));
wrench = ftip(:);
tau = zeros(n, 1);
for i = n:-1:1
    wrench = irb1300_kin_dyn.adjoint(transforms(:, :, i + 2)).' * wrench + ...
        robot.glist(:, :, i) * accelerations(:, i + 1) - ...
        irb1300_kin_dyn.ad_twist(velocities(:, i + 1)).' * (robot.glist(:, :, i) * velocities(:, i + 1));
    tau(i) = wrench.' * robot.alist(:, i);
end
end
