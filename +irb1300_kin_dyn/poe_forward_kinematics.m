function T = poe_forward_kinematics(screws, M, q)
%POE_FORWARD_KINEMATICS Product-of-exponentials forward kinematics.
n = size(screws, 2);
T = eye(4);
q = q(:);
for i = 1:n
    T = T * irb1300_kin_dyn.matrix_exp_se3(screws(:, i), q(i));
end
T = T * M;
end
