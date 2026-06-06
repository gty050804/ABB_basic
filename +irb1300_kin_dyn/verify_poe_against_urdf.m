function err = verify_poe_against_urdf(robot, q)
%VERIFY_POE_AGAINST_URDF Compare POE FK against direct URDF FK.
if nargin < 2 || isempty(q)
    q = zeros(robot.n_joints, 1);
end
T_poe = irb1300_kin_dyn.forward_kinematics(robot, q);
T_urdf = irb1300_kin_dyn.urdf_forward_kinematics(robot.joints, q, robot.base_link, robot.ee_link);
err = norm(T_poe - T_urdf, 'fro');
end
