function [q, ok, iterations] = inverse_kinematics(screws, M, target_pose, q_init, varargin)
%INVERSE_KINEMATICS Damped least-squares body-Jacobian IK.
p = inputParser;
addParameter(p, 'max_iter', 200);
addParameter(p, 'pos_tol', 1e-4);
addParameter(p, 'rot_tol', 1e-3);
addParameter(p, 'damping', 0.05);
addParameter(p, 'q_lower', []);
addParameter(p, 'q_upper', []);
parse(p, varargin{:});
opts = p.Results;

q = q_init(:);
best_q = q;
best_err = inf;
ok = false;
iterations = opts.max_iter;

for it = 1:opts.max_iter
    T_curr = irb1300_kin_dyn.poe_forward_kinematics(screws, M, q);
    twist = irb1300_kin_dyn.pose_error_twist(T_curr, target_pose);
    pos_err = norm(twist(4:6));
    rot_err = norm(twist(1:3));
    total_err = pos_err + rot_err;
    if total_err < best_err
        best_err = total_err;
        best_q = q;
    end
    if (pos_err < opts.pos_tol && rot_err < opts.rot_tol) || ...
            total_err < opts.pos_tol + opts.rot_tol
        ok = true;
        iterations = it;
        return;
    end

    J_s = irb1300_kin_dyn.space_jacobian(screws, q);
    J_b = irb1300_kin_dyn.adjoint(inv(T_curr)) * J_s;
    lam = opts.damping * (0.5 + total_err);
    dq = J_b' * ((J_b * J_b' + lam^2 * eye(6)) \ twist);
    dq = min(max(dq, -0.15), 0.15);

    alpha = 1.0;
    accepted = false;
    for ls = 1:12
        q_try = q + alpha * dq;
        if ~isempty(opts.q_lower) && ~isempty(opts.q_upper)
            q_try = min(max(q_try, opts.q_lower(:)), opts.q_upper(:));
        end
        twist_try = irb1300_kin_dyn.pose_error_twist( ...
            irb1300_kin_dyn.poe_forward_kinematics(screws, M, q_try), target_pose);
        if norm(twist_try(4:6)) + norm(twist_try(1:3)) < total_err
            q = q_try;
            accepted = true;
            break;
        end
        alpha = alpha * 0.5;
    end
    if ~accepted
        q = q + 0.05 * dq;
        if ~isempty(opts.q_lower) && ~isempty(opts.q_upper)
            q = min(max(q, opts.q_lower(:)), opts.q_upper(:));
        end
    end
end

q = best_q;
end
