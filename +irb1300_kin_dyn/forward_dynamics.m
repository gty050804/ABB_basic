function qdd = forward_dynamics(robot, q, qd, tau_drive, f_ext_ee)
%FORWARD_DYNAMICS Solve M(q) qdd = tau - bias.
if nargin < 5 || isempty(f_ext_ee)
    f_ext_ee = zeros(6, 1);
end
M = irb1300_kin_dyn.mass_matrix(robot, q);
bias = irb1300_kin_dyn.inverse_dynamics(robot, q, qd, zeros(robot.n_joints, 1), f_ext_ee);
qdd = M \ (tau_drive(:) - bias.tau_total);
end
