function T = xyz_rpy_to_transform(xyz, rpy)
%XYZ_RPY_TO_TRANSFORM Build a homogeneous transform from xyz and rpy.
T = eye(4);
T(1:3, 1:3) = irb1300_kin_dyn.rpy_to_matrix(rpy(1), rpy(2), rpy(3));
T(1:3, 4) = xyz(:);
end
