function mlist = build_mlist(joints)
%BUILD_MLIST Home transforms between adjacent link frames.
n = numel(joints);
mlist = zeros(4, 4, n + 1);
for i = 1:n
    mlist(:, :, i) = irb1300_kin_dyn.xyz_rpy_to_transform(joints(i).origin_xyz, joints(i).origin_rpy);
end
mlist(:, :, n + 1) = eye(4);
end
