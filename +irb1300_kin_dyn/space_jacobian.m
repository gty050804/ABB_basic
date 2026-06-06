function J = space_jacobian(screws, q)
%SPACE_JACOBIAN Space Jacobian for POE screw axes.
n = size(screws, 2);
J = zeros(6, n);
T = eye(4);
q = q(:);
for i = 1:n
    J(:, i) = irb1300_kin_dyn.adjoint(T) * screws(:, i);
    T = T * irb1300_kin_dyn.matrix_exp_se3(screws(:, i), q(i));
end
end
