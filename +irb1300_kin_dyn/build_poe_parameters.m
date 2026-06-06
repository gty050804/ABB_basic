function [screws, M] = build_poe_parameters(joints, base_link, ee_link)
%BUILD_POE_PARAMETERS Return space-frame screw axes and home transform.
if nargin < 2 || isempty(base_link)
    base_link = 'base_link';
end
if nargin < 3 || isempty(ee_link)
    ee_link = joints(end).child;
end
n = numel(joints);
screws = zeros(6, n);
T_accum = eye(4);
for i = 1:n
    joint = joints(i);
    T_origin = irb1300_kin_dyn.xyz_rpy_to_transform(joint.origin_xyz, joint.origin_rpy);
    T_joint_frame = T_accum * T_origin;
    w = T_joint_frame(1:3, 1:3) * joint.axis;
    q_point = T_joint_frame(1:3, 4);
    v = -cross(w, q_point);
    screws(:, i) = [w; v];
    T_accum = T_joint_frame;
end
M = irb1300_kin_dyn.urdf_forward_kinematics(joints, zeros(n, 1), base_link, ee_link);
end
