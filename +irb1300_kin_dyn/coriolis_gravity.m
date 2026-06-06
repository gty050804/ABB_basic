function c = coriolis_gravity(robot, q, qd)
%CORIOLIS_GRAVITY Bias generalized forces at zero acceleration and no wrench.
c = irb1300_kin_dyn.inverse_dynamics(robot, q, qd, zeros(robot.n_joints, 1), zeros(6, 1));
c = c.tau_total;
end
