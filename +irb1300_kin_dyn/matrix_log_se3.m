function twist = matrix_log_se3(T)
%MATRIX_LOG_SE3 Logarithm map from SE(3) to a 6-vector twist.
R = T(1:3, 1:3);
p = T(1:3, 4);
cos_angle = (trace(R) - 1) * 0.5;
cos_angle = min(max(cos_angle, -1), 1);
angle = acos(cos_angle);

if angle < 1e-8
    w = 0.5 * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
    twist = [w; p];
    return;
end

w_hat = (R - R') / (2 * sin(angle)) * angle;
w = [w_hat(3,2); w_hat(1,3); w_hat(2,1)];
wn = w / angle;
w_hat_n = irb1300_kin_dyn.skew(wn);
inv_g = eye(3) / angle - 0.5 * w_hat_n + ...
    (1 / angle - 0.5 / tan(angle / 2)) * (w_hat_n * w_hat_n);
v = inv_g * p;
twist = [w; v];
end
