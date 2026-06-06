function T = forward_kinematics(robot, q)
%FORWARD_KINEMATICS Robot forward kinematics wrapper.
T = irb1300_kin_dyn.poe_forward_kinematics(robot.screws, robot.m_home, q);
end
