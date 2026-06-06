function A = ad_twist(twist)
%AD_TWIST Lie algebra ad operator for a spatial twist.
twist = twist(:);
w = twist(1:3);
v = twist(4:6);
A = [irb1300_kin_dyn.skew(w), zeros(3, 3);
     irb1300_kin_dyn.skew(v), irb1300_kin_dyn.skew(w)];
end
