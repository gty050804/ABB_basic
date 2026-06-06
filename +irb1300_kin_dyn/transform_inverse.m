function T_inv = transform_inverse(T)
%TRANSFORM_INVERSE Invert an SE(3) homogeneous transform.
R = T(1:3, 1:3);
p = T(1:3, 4);
T_inv = eye(4);
T_inv(1:3, 1:3) = R.';
T_inv(1:3, 4) = -(R.' * p);
end
