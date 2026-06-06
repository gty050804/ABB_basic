function alist = body_screws_in_link_frames(robot)
%BODY_SCREWS_IN_LINK_FRAMES Convert space screw axes into local home frames.
n = robot.n_joints;
alist = zeros(6, n);
T_base_to_joint = eye(4);
for i = 1:n
    T_base_to_joint = T_base_to_joint * robot.mlist(:, :, i);
    alist(:, i) = irb1300_kin_dyn.adjoint( ...
        irb1300_kin_dyn.transform_inverse(T_base_to_joint)) * robot.screws(:, i);
end
end
