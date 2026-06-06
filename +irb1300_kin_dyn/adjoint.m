function Ad = adjoint(T)
%ADJOINT Return the 6-by-6 adjoint matrix of an SE(3) transform.
R = T(1:3, 1:3);
p = T(1:3, 4);
Ad = [R, zeros(3, 3);
      irb1300_kin_dyn.skew(p) * R, R];
end
