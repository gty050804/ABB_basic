function T = matrix_exp_se3(screw, theta)
%MATRIX_EXP_SE3 Exponential map for a screw axis and scalar displacement.
screw = screw(:);
w = screw(1:3);
v = screw(4:6);
w_norm = norm(w);
T = eye(4);
if w_norm < 1e-12
    T(1:3, 4) = v * theta;
    return;
end

wn = w / w_norm;
w_hat = irb1300_kin_dyn.skew(wn);
angle = w_norm * theta;
R = eye(3) + sin(angle) * w_hat + (1 - cos(angle)) * (w_hat * w_hat);
p = (eye(3) * theta + (1 - cos(angle)) * w_hat + ...
    (angle - sin(angle)) * (w_hat * w_hat)) * (v / w_norm);
T(1:3, 1:3) = R;
T(1:3, 4) = p;
end
