function twist = pose_error_twist(T_current, T_desired)
%POSE_ERROR_TWIST Body-frame pose error from current pose to desired pose.
twist = irb1300_kin_dyn.matrix_log_se3(T_current \ T_desired);
end
