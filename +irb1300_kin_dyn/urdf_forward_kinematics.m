function T = urdf_forward_kinematics(joints, q, base_link, ee_link)
%URDF_FORWARD_KINEMATICS Forward kinematics using URDF joint origins.
if nargin < 3 || isempty(base_link)
    base_link = 'base_link';
end
if nargin < 4 || isempty(ee_link)
    ee_link = joints(end).child;
end

transforms = containers.Map();
transforms(base_link) = eye(4);
for i = 1:numel(joints)
    joint = joints(i);
    T_origin = irb1300_kin_dyn.xyz_rpy_to_transform(joint.origin_xyz, joint.origin_rpy);
    w_hat = irb1300_kin_dyn.skew(joint.axis);
    R_joint = eye(3) + sin(q(i)) * w_hat + (1 - cos(q(i))) * (w_hat * w_hat);
    T_joint = eye(4);
    T_joint(1:3, 1:3) = R_joint;
    transforms(joint.child) = transforms(joint.parent) * T_origin * T_joint;
end
T = transforms(ee_link);
end
